"""
Betting Engine - Handles all betting logic, odds, and settlements
"""

import time
import uuid
from dataclasses import dataclass, asdict, field
from typing import Dict, List, Optional, Callable
from enum import Enum
from snow_ledger import ledger, Transaction


class BetType(Enum):
    COLOR_DETECTION = "color_detection"
    DISTANCE_THRESHOLD = "distance_threshold"
    SPEED_RANGE = "speed_range"
    LINE_FOLLOW_SUCCESS = "line_follow_success"
    PID_ERROR_RANGE = "pid_error_range"


class BetStatus(Enum):
    PENDING = "pending"
    ACTIVE = "active"
    WON = "won"
    LOST = "lost"
    CANCELLED = "cancelled"
    EXPIRED = "expired"


@dataclass
class BetOption:
    """A single betting option within a bet market"""
    option_id: str
    name: str
    description: str
    odds: float  # Decimal odds (e.g., 2.0 = 2:1)
    
    def to_dict(self):
        return asdict(self)


@dataclass
class BetMarket:
    """A betting market with multiple options"""
    market_id: str
    bet_type: BetType
    title: str
    description: str
    options: List[BetOption]
    created_at: float
    expires_at: float
    status: str = "open"  # open, closed, settled
    winning_option_id: Optional[str] = None
    
    def to_dict(self):
        return {
            **asdict(self),
            "bet_type": self.bet_type.value,
            "options": [opt.to_dict() for opt in self.options]
        }


@dataclass
class PlacedBet:
    """A bet placed by a user"""
    bet_id: str
    user_address: str
    market_id: str
    option_id: str
    amount: float
    odds_at_placement: float
    potential_payout: float
    placed_at: float
    status: BetStatus = BetStatus.PENDING
    settled_at: Optional[float] = None
    payout: float = 0.0
    
    def to_dict(self):
        return {
            **asdict(self),
            "status": self.status.value
        }


class BettingEngine:
    """
    Core betting engine handling:
    - Market creation and management
    - Bet placement and validation
    - Settlement and payouts
    - Odds calculation
    """
    
    def __init__(self):
        self.markets: Dict[str, BetMarket] = {}
        self.placed_bets: Dict[str, PlacedBet] = {}
        self.user_bets: Dict[str, List[str]] = {}  # user_address -> [bet_ids]
        
        # Initialize default markets
        self._create_default_markets()
    
    def _generate_id(self) -> str:
        return str(uuid.uuid4())[:8]
    
    def _create_default_markets(self):
        """Create the default betting markets based on robot sensors"""
        
        # Color Detection Market
        self.create_market(
            bet_type=BetType.COLOR_DETECTION,
            title="Next Color Detection",
            description="What color will the robot detect next?",
            options=[
                BetOption("red", "Red Line", "Robot detects red line", 1.8),
                BetOption("white", "White Surface", "Robot detects white background", 2.2),
                BetOption("dark_red", "Dark Red", "Robot detects dark red line", 3.5),
                BetOption("unknown", "Unknown/Error", "Sensor returns unexpected value", 10.0),
            ],
            duration_seconds=60
        )
        
        # Distance Threshold Market
        self.create_market(
            bet_type=BetType.DISTANCE_THRESHOLD,
            title="Distance Reading",
            description="Will ultrasonic sensor read above or below 20cm?",
            options=[
                BetOption("above", "Above 20cm", "Distance > 20cm", 1.9),
                BetOption("below", "Below 20cm", "Distance ≤ 20cm", 1.9),
                BetOption("no_reading", "No Reading", "Sensor timeout/error", 8.0),
            ],
            duration_seconds=30
        )
        
        # Speed Range Market
        self.create_market(
            bet_type=BetType.SPEED_RANGE,
            title="Robot Speed",
            description="What speed range will the robot operate at?",
            options=[
                BetOption("slow", "Slow (0-80)", "PWM speed 0-80", 2.5),
                BetOption("medium", "Medium (81-150)", "PWM speed 81-150", 1.8),
                BetOption("fast", "Fast (151-255)", "PWM speed 151-255", 2.2),
                BetOption("stopped", "Stopped", "Robot not moving", 4.0),
            ],
            duration_seconds=45
        )
        
        # Line Follow Success Market
        self.create_market(
            bet_type=BetType.LINE_FOLLOW_SUCCESS,
            title="Line Follow Challenge",
            description="Will robot stay on line for next 10 seconds?",
            options=[
                BetOption("success", "Stays On Line", "Successfully follows line", 1.6),
                BetOption("fail", "Loses Line", "Robot goes off track", 2.4),
            ],
            duration_seconds=15
        )
        
        # PID Error Market
        self.create_market(
            bet_type=BetType.PID_ERROR_RANGE,
            title="PID Error Range",
            description="What will the average PID error be?",
            options=[
                BetOption("low", "Low Error (0-30)", "Well-tuned, smooth", 2.0),
                BetOption("medium", "Medium Error (31-60)", "Some oscillation", 1.8),
                BetOption("high", "High Error (61-100)", "Aggressive corrections", 2.5),
            ],
            duration_seconds=30
        )
    
    def create_market(self, bet_type: BetType, title: str, description: str,
                      options: List[BetOption], duration_seconds: int) -> BetMarket:
        """Create a new betting market"""
        market_id = self._generate_id()
        now = time.time()
        
        market = BetMarket(
            market_id=market_id,
            bet_type=bet_type,
            title=title,
            description=description,
            options=options,
            created_at=now,
            expires_at=now + duration_seconds,
            status="open"
        )
        
        self.markets[market_id] = market
        return market
    
    def get_market(self, market_id: str) -> Optional[BetMarket]:
        return self.markets.get(market_id)
    
    def get_open_markets(self) -> List[BetMarket]:
        """Get all open markets that haven't expired"""
        now = time.time()
        return [
            m for m in self.markets.values()
            if m.status == "open" and m.expires_at > now
        ]
    
    def get_all_markets(self) -> List[BetMarket]:
        """Get all markets"""
        return list(self.markets.values())
    
    def place_bet(self, user_address: str, market_id: str, 
                  option_id: str, amount: float) -> Optional[PlacedBet]:
        """
        Place a bet on a market option
        Returns PlacedBet if successful, None if failed
        """
        # Validate market
        market = self.markets.get(market_id)
        if not market or market.status != "open":
            return None
        
        if time.time() > market.expires_at:
            market.status = "closed"
            return None
        
        # Validate option
        option = next((o for o in market.options if o.option_id == option_id), None)
        if not option:
            return None
        
        # Validate amount
        if amount < 1.0:
            return None
        
        # Deduct from user's balance
        tx = ledger.place_bet(
            address=user_address,
            amount=amount,
            bet_memo=f"Bet on {market.title}: {option.name}"
        )
        
        if not tx:
            return None  # Insufficient balance or invalid address
        
        # Create placed bet
        bet_id = self._generate_id()
        placed_bet = PlacedBet(
            bet_id=bet_id,
            user_address=user_address,
            market_id=market_id,
            option_id=option_id,
            amount=amount,
            odds_at_placement=option.odds,
            potential_payout=amount * option.odds,
            placed_at=time.time(),
            status=BetStatus.ACTIVE
        )
        
        self.placed_bets[bet_id] = placed_bet
        
        # Track user's bets
        if user_address not in self.user_bets:
            self.user_bets[user_address] = []
        self.user_bets[user_address].append(bet_id)
        
        return placed_bet
    
    def settle_market(self, market_id: str, winning_option_id: str) -> List[PlacedBet]:
        """
        Settle a market with the winning option
        Returns list of settled bets
        """
        market = self.markets.get(market_id)
        if not market:
            return []
        
        market.status = "settled"
        market.winning_option_id = winning_option_id
        
        settled_bets = []
        
        # Process all bets on this market
        for bet in self.placed_bets.values():
            if bet.market_id != market_id or bet.status != BetStatus.ACTIVE:
                continue
            
            bet.settled_at = time.time()
            
            if bet.option_id == winning_option_id:
                # Winner!
                bet.status = BetStatus.WON
                bet.payout = bet.potential_payout
                
                # Pay out winnings
                ledger.pay_winnings(
                    address=bet.user_address,
                    amount=bet.payout,
                    bet_memo=f"Won bet on {market.title}"
                )
            else:
                # Loser
                bet.status = BetStatus.LOST
                bet.payout = 0.0
            
            settled_bets.append(bet)
        
        return settled_bets
    
    def get_user_bets(self, user_address: str) -> List[PlacedBet]:
        """Get all bets for a user"""
        bet_ids = self.user_bets.get(user_address, [])
        return [self.placed_bets[bid] for bid in bet_ids if bid in self.placed_bets]
    
    def get_user_active_bets(self, user_address: str) -> List[PlacedBet]:
        """Get active bets for a user"""
        return [b for b in self.get_user_bets(user_address) if b.status == BetStatus.ACTIVE]
    
    def get_market_bets(self, market_id: str) -> List[PlacedBet]:
        """Get all bets on a market"""
        return [b for b in self.placed_bets.values() if b.market_id == market_id]
    
    def refresh_market(self, market_id: str):
        """Refresh/recreate an expired market"""
        old_market = self.markets.get(market_id)
        if not old_market:
            return None
        
        # Create new market with same parameters
        return self.create_market(
            bet_type=old_market.bet_type,
            title=old_market.title,
            description=old_market.description,
            options=old_market.options,
            duration_seconds=60  # Default refresh duration
        )
    
    def get_stats(self) -> dict:
        """Get betting statistics"""
        total_bets = len(self.placed_bets)
        total_wagered = sum(b.amount for b in self.placed_bets.values())
        total_paid_out = sum(b.payout for b in self.placed_bets.values() if b.status == BetStatus.WON)
        
        return {
            "total_markets": len(self.markets),
            "open_markets": len(self.get_open_markets()),
            "total_bets": total_bets,
            "total_wagered": total_wagered,
            "total_paid_out": total_paid_out,
            "house_edge": total_wagered - total_paid_out if total_wagered > 0 else 0
        }


# Global betting engine instance
betting_engine = BettingEngine()
