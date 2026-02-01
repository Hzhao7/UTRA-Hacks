"""
Snowbets - Main Flask Application
Real-time robot betting platform with SocketIO
"""

from flask import Flask, render_template, request, jsonify, session
from flask_socketio import SocketIO, emit, join_room, leave_room
from flask_cors import CORS
import time
import threading

from snow_ledger import ledger
from betting_engine import betting_engine, BetStatus
from robot_interface import robot

app = Flask(__name__)
app.secret_key = 'snowbets_secret_key_change_in_production'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Simulation mode flag
SIMULATION_MODE = True  # Set to False when real robot is connected


# ============== Helper Functions ==============

def get_current_user():
    """Get current user from session"""
    if 'user_address' not in session:
        return None
    return ledger.get_account(session['user_address'])


# ============== HTTP Routes ==============

@app.route('/')
def index():
    """Main betting page"""
    return render_template('index.html')


@app.route('/api/register', methods=['POST'])
def register():
    """Register a new user"""
    data = request.get_json()
    username = data.get('username', '').strip()
    
    if not username or len(username) < 3:
        return jsonify({'error': 'Username must be at least 3 characters'}), 400
    
    account = ledger.create_account(username)
    if not account:
        return jsonify({'error': 'Username already exists'}), 400
    
    session['user_address'] = account.address
    session['username'] = account.username
    
    return jsonify({
        'success': True,
        'account': account.to_dict()
    })


@app.route('/api/login', methods=['POST'])
def login():
    """Login existing user"""
    data = request.get_json()
    username = data.get('username', '').strip()
    
    account = ledger.get_account_by_username(username)
    if not account:
        return jsonify({'error': 'User not found'}), 404
    
    session['user_address'] = account.address
    session['username'] = account.username
    
    return jsonify({
        'success': True,
        'account': account.to_dict()
    })


@app.route('/api/logout', methods=['POST'])
def logout():
    """Logout current user"""
    session.clear()
    return jsonify({'success': True})


@app.route('/api/me')
def get_me():
    """Get current user info"""
    user = get_current_user()
    if not user:
        return jsonify({'error': 'Not logged in'}), 401
    
    return jsonify({
        'account': user.to_dict(),
        'bets': [b.to_dict() for b in betting_engine.get_user_bets(user.address)]
    })


@app.route('/api/robot')
def get_robot_data():
    """Get current robot data (for polling fallback)"""
    if SIMULATION_MODE:
        robot.simulate_data()
    
    data = robot.get_current_data()
    return jsonify({
        'robot': data.to_dict(),
        'speed_category': robot.get_speed_category(),
        'on_line': robot.is_on_line(),
        'avg_error': robot.get_average_error(5.0),
        'timestamp': time.time()
    })


@app.route('/api/balance')
def get_balance():
    """Get current user's Snow balance"""
    user = get_current_user()
    if not user:
        return jsonify({'error': 'Not logged in'}), 401
    
    return jsonify({'balance': user.balance})


@app.route('/api/markets')
def get_markets():
    """Get all open betting markets"""
    markets = betting_engine.get_open_markets()
    return jsonify({
        'markets': [m.to_dict() for m in markets]
    })


@app.route('/api/markets/<market_id>')
def get_market(market_id):
    """Get specific market details"""
    market = betting_engine.get_market(market_id)
    if not market:
        return jsonify({'error': 'Market not found'}), 404
    
    bets = betting_engine.get_market_bets(market_id)
    return jsonify({
        'market': market.to_dict(),
        'total_bets': len(bets),
        'total_wagered': sum(b.amount for b in bets)
    })


@app.route('/api/bet', methods=['POST'])
def place_bet():
    """Place a bet"""
    user = get_current_user()
    if not user:
        return jsonify({'error': 'Not logged in'}), 401
    
    data = request.get_json()
    market_id = data.get('market_id')
    option_id = data.get('option_id')
    amount = float(data.get('amount', 0))
    
    if amount < 1:
        return jsonify({'error': 'Minimum bet is 1 Snow'}), 400
    
    if amount > user.balance:
        return jsonify({'error': 'Insufficient Snow balance'}), 400
    
    placed_bet = betting_engine.place_bet(
        user_address=user.address,
        market_id=market_id,
        option_id=option_id,
        amount=amount
    )
    
    if not placed_bet:
        return jsonify({'error': 'Failed to place bet'}), 400
    
    # Broadcast bet placed
    socketio.emit('bet_placed', {
        'market_id': market_id,
        'bet': placed_bet.to_dict()
    })
    
    return jsonify({
        'success': True,
        'bet': placed_bet.to_dict(),
        'new_balance': ledger.get_balance(user.address)
    })


@app.route('/api/my-bets')
def get_my_bets():
    """Get current user's bets"""
    user = get_current_user()
    if not user:
        return jsonify({'error': 'Not logged in'}), 401
    
    bets = betting_engine.get_user_bets(user.address)
    return jsonify({
        'bets': [b.to_dict() for b in bets]
    })


@app.route('/api/leaderboard')
def get_leaderboard():
    """Get top users by balance"""
    leaders = ledger.get_leaderboard(10)
    return jsonify({
        'leaderboard': [l.to_dict() for l in leaders]
    })


@app.route('/api/transactions')
def get_transactions():
    """Get current user's transaction history"""
    user = get_current_user()
    if not user:
        return jsonify({'error': 'Not logged in'}), 401
    
    txs = ledger.get_transaction_history(user.address, 50)
    return jsonify({
        'transactions': [t.to_dict() for t in txs]
    })


@app.route('/api/stats')
def get_stats():
    """Get platform statistics"""
    return jsonify({
        'betting': betting_engine.get_stats(),
        'ledger': {
            'total_accounts': len(ledger.accounts) - 1,  # Exclude house
            'total_transactions': len(ledger.transactions),
            'block_height': ledger.block_height
        }
    })


# ============== SocketIO Events ==============

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print(f"Client connected: {request.sid}")
    emit('connected', {'status': 'ok'})


@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print(f"Client disconnected: {request.sid}")


@socketio.on('subscribe_robot')
def handle_subscribe_robot():
    """Subscribe to robot data updates"""
    join_room('robot_data')
    emit('subscribed', {'room': 'robot_data'})


@socketio.on('subscribe_markets')
def handle_subscribe_markets():
    """Subscribe to market updates"""
    join_room('markets')
    emit('subscribed', {'room': 'markets'})


# ============== Background Tasks ==============

def robot_data_broadcast():
    """Broadcast robot data to subscribers"""
    while True:
        if SIMULATION_MODE:
            robot.simulate_data()
        
        data = robot.get_current_data()
        socketio.emit('robot_update', {
            'robot': data.to_dict(),
            'speed_category': robot.get_speed_category(),
            'on_line': robot.is_on_line(),
            'avg_error': robot.get_average_error(5.0),
            'timestamp': time.time()
        })  # Broadcast to all connected clients
        
        socketio.sleep(1.0)  # Use socketio.sleep for proper async


def market_settlement_loop():
    """Check and settle expired markets, create new ones"""
    while True:
        now = time.time()
        
        for market in betting_engine.get_all_markets():
            # Check if market expired and needs settlement
            if market.status == "open" and now > market.expires_at:
                market.status = "closed"
                
                # Determine winner based on current robot state
                winning_option = determine_winner(market)
                
                if winning_option:
                    settled_bets = betting_engine.settle_market(market.market_id, winning_option)
                    
                    # Broadcast settlement
                    socketio.emit('market_settled', {
                        'market_id': market.market_id,
                        'winning_option': winning_option,
                        'settled_bets': len(settled_bets)
                    }, room='markets')
                    
                    # Notify individual winners
                    for bet in settled_bets:
                        if bet.status == BetStatus.WON:
                            socketio.emit('bet_won', {
                                'bet': bet.to_dict(),
                                'new_balance': ledger.get_balance(bet.user_address)
                            })
                
                # Create new market of same type
                betting_engine.refresh_market(market.market_id)
        
        # Broadcast market list update
        socketio.emit('markets_update', {
            'markets': [m.to_dict() for m in betting_engine.get_open_markets()]
        })
        
        socketio.sleep(5)  # Check every 5 seconds


def determine_winner(market) -> str:
    """Determine winning option based on robot state"""
    robot_data = robot.get_current_data()
    bet_type = market.bet_type.value
    
    if bet_type == "color_detection":
        return robot_data.detected_color
    
    elif bet_type == "distance_threshold":
        if robot_data.distance == 0:
            return "no_reading"
        elif robot_data.distance > 20:
            return "above"
        else:
            return "below"
    
    elif bet_type == "speed_range":
        return robot.get_speed_category()
    
    elif bet_type == "line_follow_success":
        return "success" if robot.is_on_line() else "fail"
    
    elif bet_type == "pid_error_range":
        avg_error = robot.get_average_error(5.0)
        if avg_error <= 30:
            return "low"
        elif avg_error <= 60:
            return "medium"
        else:
            return "high"
    
    return None


# ============== Main ==============

if __name__ == '__main__':
    print("=" * 50)
    print("  SNOWBETS - Robot Betting Platform")
    print("=" * 50)
    print(f"  Simulation Mode: {SIMULATION_MODE}")
    print("  Starting server on http://localhost:5000")
    print("=" * 50)
    
    # Start background tasks using socketio for proper eventlet compatibility
    socketio.start_background_task(robot_data_broadcast)
    socketio.start_background_task(market_settlement_loop)
    
    # Run Flask app
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False)
