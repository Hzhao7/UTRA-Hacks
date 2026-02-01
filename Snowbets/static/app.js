/**
 * Snowbets - Frontend Application
 * Real-time betting interface with SocketIO
 */

// ============== State ==============
let currentUser = null;
let socket = null;
let markets = [];
let selectedMarket = null;
let selectedOption = null;

// ============== DOM Elements ==============
const authModal = document.getElementById('authModal');
const betModal = document.getElementById('betModal');
const usernameInput = document.getElementById('usernameInput');
const balanceAmount = document.getElementById('balanceAmount');
const usernameDisplay = document.getElementById('username');
const logoutBtn = document.getElementById('logoutBtn');
const marketsGrid = document.getElementById('marketsGrid');
const myBetsContainer = document.getElementById('myBets');
const leaderboardContainer = document.getElementById('leaderboard');
const toastContainer = document.getElementById('toastContainer');

// Robot data elements
const detectedColor = document.getElementById('detectedColor');
const colorPreview = document.getElementById('colorPreview');
const redValue = document.getElementById('redValue');
const greenValue = document.getElementById('greenValue');
const blueValue = document.getElementById('blueValue');
const speedCategory = document.getElementById('speedCategory');
const speedFill = document.getElementById('speedFill');
const pidError = document.getElementById('pidError');
const onLine = document.getElementById('onLine');
const leftSpeed = document.getElementById('leftSpeed');
const rightSpeed = document.getElementById('rightSpeed');

// ============== Initialization ==============
document.addEventListener('DOMContentLoaded', () => {
    initSocket();
    checkAuth();
    loadMarkets();
    loadLeaderboard();
    loadStats();
    setupEventListeners();
});

function initSocket() {
    socket = io();
    
    socket.on('connect', () => {
        console.log('Connected to Snowbets server');
        socket.emit('subscribe_robot');
        socket.emit('subscribe_markets');
    });
    
    socket.on('robot_update', (data) => {
        updateRobotDisplay(data);
    });
    
    socket.on('markets_update', (data) => {
        markets = data.markets;
        renderMarkets();
    });
    
    socket.on('bet_placed', (data) => {
        // Refresh bets if it's our bet
        if (currentUser) {
            loadMyBets();
        }
    });
    
    socket.on('market_settled', (data) => {
        showToast(`Market settled! Winner: ${data.winning_option}`, 'info');
        loadMarkets();
    });
    
    socket.on('bet_won', (data) => {
        if (currentUser && data.bet.user_address === currentUser.address) {
            showToast(`You won ${data.bet.payout.toFixed(2)} Snow!`, 'win');
            updateBalance(data.new_balance);
            loadMyBets();
        }
    });
}

// ============== Authentication ==============
async function checkAuth() {
    try {
        const response = await fetch('/api/me');
        if (response.ok) {
            const data = await response.json();
            currentUser = data.account;
            onLogin();
        }
    } catch (e) {
        console.log('Not logged in');
    }
}

function onLogin() {
    authModal.style.display = 'none';
    usernameDisplay.textContent = currentUser.username;
    updateBalance(currentUser.balance);
    logoutBtn.style.display = 'block';
    loadMyBets();
}

function onLogout() {
    currentUser = null;
    authModal.style.display = 'flex';
    usernameDisplay.textContent = 'Guest';
    balanceAmount.textContent = '0';
    logoutBtn.style.display = 'none';
    myBetsContainer.innerHTML = '<p class="empty-state">No active bets</p>';
}

function updateBalance(balance) {
    balanceAmount.textContent = balance.toFixed(2);
}

async function login() {
    const username = usernameInput.value.trim();
    if (username.length < 3) {
        showToast('Username must be at least 3 characters', 'error');
        return;
    }
    
    try {
        const response = await fetch('/api/login', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ username })
        });
        
        const data = await response.json();
        
        if (response.ok) {
            currentUser = data.account;
            onLogin();
            showToast(`Welcome back, ${currentUser.username}!`, 'success');
        } else {
            showToast(data.error, 'error');
        }
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function register() {
    const username = usernameInput.value.trim();
    if (username.length < 3) {
        showToast('Username must be at least 3 characters', 'error');
        return;
    }
    
    try {
        const response = await fetch('/api/register', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ username })
        });
        
        const data = await response.json();
        
        if (response.ok) {
            currentUser = data.account;
            onLogin();
            showToast(`Welcome ${currentUser.username}! You received 1,000 Snow!`, 'success');
        } else {
            showToast(data.error, 'error');
        }
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function logout() {
    await fetch('/api/logout', { method: 'POST' });
    onLogout();
}

// ============== Robot Display ==============
function updateRobotDisplay(data) {
    const robot = data.robot;
    
    // Color
    detectedColor.textContent = robot.detected_color.toUpperCase();
    updateColorPreview(robot.detected_color);
    
    // RGB
    redValue.textContent = `R: ${robot.red}`;
    greenValue.textContent = `G: ${robot.green}`;
    blueValue.textContent = `B: ${robot.blue}`;
    
    // Speed
    speedCategory.textContent = data.speed_category.toUpperCase();
    const avgSpeed = (robot.left_speed + robot.right_speed) / 2;
    speedFill.style.width = `${(avgSpeed / 255) * 100}%`;
    
    // PID
    pidError.textContent = robot.error.toFixed(1);
    
    // On Line
    onLine.textContent = data.on_line ? 'YES' : 'NO';
    onLine.className = `stat-value status-indicator ${data.on_line ? 'on-line' : 'off-line'}`;
    
    // Motor speeds
    leftSpeed.textContent = `L: ${robot.left_speed}`;
    rightSpeed.textContent = `R: ${robot.right_speed}`;
}

function updateColorPreview(color) {
    const colors = {
        'red': '#dc2626',
        'dark_red': '#7f1d1d',
        'white': '#ffffff',
        'unknown': '#6b7280'
    };
    colorPreview.style.backgroundColor = colors[color] || colors['unknown'];
}

// ============== Markets ==============
async function loadMarkets() {
    try {
        const response = await fetch('/api/markets');
        const data = await response.json();
        markets = data.markets;
        renderMarkets();
    } catch (e) {
        console.error('Failed to load markets', e);
    }
}

function renderMarkets() {
    marketsGrid.innerHTML = markets.map(market => `
        <div class="market-card" onclick="openBetModal('${market.market_id}')">
            <div class="market-title">${market.title}</div>
            <div class="market-description">${market.description}</div>
            <div class="market-timer">Expires in ${formatTimeRemaining(market.expires_at)}</div>
            <div class="market-options">
                ${market.options.map(opt => `
                    <div class="option-chip">
                        <span>${opt.name}</span>
                        <span class="option-odds">${opt.odds}x</span>
                    </div>
                `).join('')}
            </div>
        </div>
    `).join('');
}

function formatTimeRemaining(expiresAt) {
    const remaining = Math.max(0, expiresAt - Date.now() / 1000);
    const minutes = Math.floor(remaining / 60);
    const seconds = Math.floor(remaining % 60);
    return `${minutes}:${seconds.toString().padStart(2, '0')}`;
}

// ============== Betting ==============
function openBetModal(marketId) {
    if (!currentUser) {
        showToast('Please login to place bets', 'error');
        return;
    }
    
    selectedMarket = markets.find(m => m.market_id === marketId);
    if (!selectedMarket) return;
    
    selectedOption = null;
    
    document.getElementById('betModalTitle').textContent = selectedMarket.title;
    document.getElementById('betModalDescription').textContent = selectedMarket.description;
    
    const optionsContainer = document.getElementById('betOptions');
    optionsContainer.innerHTML = selectedMarket.options.map(opt => `
        <div class="bet-option" data-option="${opt.option_id}" data-odds="${opt.odds}" onclick="selectOption(this)">
            <div>
                <div class="bet-option-name">${opt.name}</div>
                <div class="bet-option-desc">${opt.description}</div>
            </div>
            <div class="bet-option-odds">${opt.odds}x</div>
        </div>
    `).join('');
    
    updatePotentialPayout();
    betModal.style.display = 'flex';
}

function selectOption(element) {
    document.querySelectorAll('.bet-option').forEach(el => el.classList.remove('selected'));
    element.classList.add('selected');
    selectedOption = {
        id: element.dataset.option,
        odds: parseFloat(element.dataset.odds)
    };
    updatePotentialPayout();
}

function updatePotentialPayout() {
    const amount = parseFloat(document.getElementById('betAmount').value) || 0;
    const odds = selectedOption ? selectedOption.odds : 0;
    const payout = amount * odds;
    document.getElementById('potentialPayout').textContent = payout.toFixed(2);
}

async function placeBet() {
    if (!selectedOption) {
        showToast('Please select an option', 'error');
        return;
    }
    
    const amount = parseFloat(document.getElementById('betAmount').value);
    if (amount < 1) {
        showToast('Minimum bet is 1 Snow', 'error');
        return;
    }
    
    if (amount > currentUser.balance) {
        showToast('Insufficient Snow balance', 'error');
        return;
    }
    
    try {
        const response = await fetch('/api/bet', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                market_id: selectedMarket.market_id,
                option_id: selectedOption.id,
                amount: amount
            })
        });
        
        const data = await response.json();
        
        if (response.ok) {
            showToast(`Bet placed! Potential win: ${data.bet.potential_payout.toFixed(2)} Snow`, 'success');
            updateBalance(data.new_balance);
            currentUser.balance = data.new_balance;
            closeBetModal();
            loadMyBets();
        } else {
            showToast(data.error, 'error');
        }
    } catch (e) {
        showToast('Failed to place bet', 'error');
    }
}

function closeBetModal() {
    betModal.style.display = 'none';
    selectedMarket = null;
    selectedOption = null;
}

// ============== My Bets ==============
async function loadMyBets() {
    if (!currentUser) return;
    
    try {
        const response = await fetch('/api/my-bets');
        const data = await response.json();
        renderMyBets(data.bets);
    } catch (e) {
        console.error('Failed to load bets', e);
    }
}

function renderMyBets(bets) {
    if (bets.length === 0) {
        myBetsContainer.innerHTML = '<p class="empty-state">No bets yet</p>';
        return;
    }
    
    // Show most recent first
    const recentBets = bets.slice(-10).reverse();
    
    myBetsContainer.innerHTML = recentBets.map(bet => {
        const market = markets.find(m => m.market_id === bet.market_id);
        const marketTitle = market ? market.title : 'Unknown Market';
        
        return `
            <div class="bet-item ${bet.status}">
                <div class="bet-market">${marketTitle}</div>
                <div class="bet-details">
                    <span>${bet.amount} Snow on ${bet.option_id}</span>
                    <span>${bet.status === 'won' ? `Won ${bet.payout} Snow` : bet.status.toUpperCase()}</span>
                </div>
            </div>
        `;
    }).join('');
}

// ============== Leaderboard ==============
async function loadLeaderboard() {
    try {
        const response = await fetch('/api/leaderboard');
        const data = await response.json();
        renderLeaderboard(data.leaderboard);
    } catch (e) {
        console.error('Failed to load leaderboard', e);
    }
}

function renderLeaderboard(leaders) {
    leaderboardContainer.innerHTML = leaders.map((user, i) => `
        <div class="leader-item">
            <span class="leader-rank">#${i + 1}</span>
            <span class="leader-name">${user.username}</span>
            <span class="leader-balance">${user.balance.toFixed(0)}</span>
        </div>
    `).join('');
}

// ============== Stats ==============
async function loadStats() {
    try {
        const response = await fetch('/api/stats');
        const data = await response.json();
        document.getElementById('totalWagered').textContent = `${data.betting.total_wagered.toFixed(0)}`;
        document.getElementById('activeBets').textContent = data.betting.total_bets;
    } catch (e) {
        console.error('Failed to load stats', e);
    }
}

// ============== Toast Notifications ==============
function showToast(message, type = 'info') {
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.textContent = message;
    toastContainer.appendChild(toast);
    
    setTimeout(() => {
        toast.remove();
    }, 4000);
}

// ============== Event Listeners ==============
function setupEventListeners() {
    // Auth
    document.getElementById('loginBtn').addEventListener('click', login);
    document.getElementById('registerBtn').addEventListener('click', register);
    logoutBtn.addEventListener('click', logout);
    
    usernameInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') login();
    });
    
    // Bet modal
    document.getElementById('confirmBetBtn').addEventListener('click', placeBet);
    document.getElementById('cancelBetBtn').addEventListener('click', closeBetModal);
    document.getElementById('betAmount').addEventListener('input', updatePotentialPayout);
    
    // Quick amounts
    document.querySelectorAll('.quick-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.getElementById('betAmount').value = btn.dataset.amount;
            updatePotentialPayout();
        });
    });
    
    // Periodic refresh
    setInterval(() => {
        loadLeaderboard();
        loadStats();
        renderMarkets(); // Update timers
    }, 5000);
}
