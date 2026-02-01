"""
Snow Ledger - Solana-inspired virtual currency system
Tracks Snow balances and transactions with cryptographic verification
"""

import hashlib
import time
import json
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional
import base58

@dataclass
class Transaction:
    """A single Snow transaction"""
    tx_id: str
    from_address: str
    to_address: str
    amount: float
    timestamp: float
    tx_type: str  # 'transfer', 'bet_place', 'bet_win', 'bet_loss', 'airdrop'
    memo: str = ""
    signature: str = ""
    
    def to_dict(self):
        return asdict(self)

@dataclass
class Account:
    """A Snow wallet account"""
    address: str
    balance: float
    username: str
    created_at: float
    transaction_count: int = 0
    
    def to_dict(self):
        return asdict(self)

class SnowLedger:
    """
    Solana-inspired ledger for Snow virtual currency
    - Tracks all accounts and balances
    - Records all transactions with verification
    - Provides audit trail
    """
    
    HOUSE_ADDRESS = "SNOW_HOUSE_11111111111111111111111111111111"
    INITIAL_AIRDROP = 1000.0  # Starting Snow for new users
    
    def __init__(self):
        self.accounts: Dict[str, Account] = {}
        self.transactions: List[Transaction] = []
        self.block_height = 0
        
        # Initialize house account with unlimited Snow
        self._create_house_account()
    
    def _create_house_account(self):
        """Create the house/bank account"""
        self.accounts[self.HOUSE_ADDRESS] = Account(
            address=self.HOUSE_ADDRESS,
            balance=1_000_000_000.0,  # 1 billion Snow
            username="HOUSE",
            created_at=time.time()
        )
    
    def _generate_address(self, username: str) -> str:
        """Generate a Solana-style base58 address from username"""
        hash_input = f"{username}:{time.time()}:{len(self.accounts)}"
        hash_bytes = hashlib.sha256(hash_input.encode()).digest()
        address = base58.b58encode(hash_bytes).decode()[:44]
        return f"SNOW_{address}"
    
    def _generate_tx_id(self, tx_data: str) -> str:
        """Generate transaction ID"""
        hash_input = f"{tx_data}:{time.time()}:{self.block_height}"
        hash_bytes = hashlib.sha256(hash_input.encode()).digest()
        return base58.b58encode(hash_bytes).decode()[:64]
    
    def _sign_transaction(self, tx: Transaction) -> str:
        """Create a signature for the transaction"""
        tx_string = f"{tx.from_address}:{tx.to_address}:{tx.amount}:{tx.timestamp}"
        return hashlib.sha256(tx_string.encode()).hexdigest()[:32]
    
    def create_account(self, username: str) -> Optional[Account]:
        """Create a new Snow account with initial airdrop"""
        # Check if username already exists
        for account in self.accounts.values():
            if account.username.lower() == username.lower():
                return None
        
        address = self._generate_address(username)
        account = Account(
            address=address,
            balance=0.0,
            username=username,
            created_at=time.time()
        )
        self.accounts[address] = account
        
        # Airdrop initial Snow
        self.transfer(
            from_address=self.HOUSE_ADDRESS,
            to_address=address,
            amount=self.INITIAL_AIRDROP,
            tx_type="airdrop",
            memo=f"Welcome airdrop for {username}"
        )
        
        return account
    
    def get_account(self, address: str) -> Optional[Account]:
        """Get account by address"""
        return self.accounts.get(address)
    
    def get_account_by_username(self, username: str) -> Optional[Account]:
        """Get account by username"""
        for account in self.accounts.values():
            if account.username.lower() == username.lower():
                return account
        return None
    
    def get_balance(self, address: str) -> float:
        """Get balance for an address"""
        account = self.accounts.get(address)
        return account.balance if account else 0.0
    
    def transfer(self, from_address: str, to_address: str, amount: float,
                 tx_type: str = "transfer", memo: str = "") -> Optional[Transaction]:
        """
        Transfer Snow between accounts
        Returns transaction if successful, None if failed
        """
        # Validate addresses
        from_account = self.accounts.get(from_address)
        to_account = self.accounts.get(to_address)
        
        if not from_account or not to_account:
            return None
        
        # Check balance (house has unlimited)
        if from_address != self.HOUSE_ADDRESS and from_account.balance < amount:
            return None
        
        if amount <= 0:
            return None
        
        # Create transaction
        tx = Transaction(
            tx_id=self._generate_tx_id(f"{from_address}:{to_address}:{amount}"),
            from_address=from_address,
            to_address=to_address,
            amount=amount,
            timestamp=time.time(),
            tx_type=tx_type,
            memo=memo
        )
        tx.signature = self._sign_transaction(tx)
        
        # Execute transfer
        if from_address != self.HOUSE_ADDRESS:
            from_account.balance -= amount
        from_account.transaction_count += 1
        
        to_account.balance += amount
        to_account.transaction_count += 1
        
        # Record transaction
        self.transactions.append(tx)
        self.block_height += 1
        
        return tx
    
    def place_bet(self, address: str, amount: float, bet_memo: str) -> Optional[Transaction]:
        """Deduct Snow for placing a bet"""
        return self.transfer(
            from_address=address,
            to_address=self.HOUSE_ADDRESS,
            amount=amount,
            tx_type="bet_place",
            memo=bet_memo
        )
    
    def pay_winnings(self, address: str, amount: float, bet_memo: str) -> Optional[Transaction]:
        """Pay out winnings to user"""
        return self.transfer(
            from_address=self.HOUSE_ADDRESS,
            to_address=address,
            amount=amount,
            tx_type="bet_win",
            memo=bet_memo
        )
    
    def get_transaction_history(self, address: str, limit: int = 50) -> List[Transaction]:
        """Get transaction history for an address"""
        history = [
            tx for tx in self.transactions
            if tx.from_address == address or tx.to_address == address
        ]
        return sorted(history, key=lambda x: x.timestamp, reverse=True)[:limit]
    
    def get_leaderboard(self, limit: int = 10) -> List[Account]:
        """Get top accounts by balance"""
        user_accounts = [
            acc for addr, acc in self.accounts.items()
            if addr != self.HOUSE_ADDRESS
        ]
        return sorted(user_accounts, key=lambda x: x.balance, reverse=True)[:limit]
    
    def export_ledger(self) -> dict:
        """Export entire ledger state"""
        return {
            "block_height": self.block_height,
            "accounts": {addr: acc.to_dict() for addr, acc in self.accounts.items()},
            "transactions": [tx.to_dict() for tx in self.transactions],
            "timestamp": time.time()
        }


# Global ledger instance
ledger = SnowLedger()
