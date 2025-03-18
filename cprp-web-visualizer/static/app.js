let chart;
let updateInterval;
const UPDATE_INTERVAL_MS = 100;
const MAX_POINTS = 1000;

// Initialize the chart with empty data
function initChart() {
    const traces = [];
    const colors = [
        '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
        '#9467bd', '#8c564b', '#e377c2', '#7f7f7f',
        '#bcbd22', '#17becf', '#aec7e8', '#ffbb78'
    ];

    // Create 12 traces, one for each ADC channel
    for (let i = 0; i < 12; i++) {
        traces.push({
            x: [],
            y: [],
            mode: 'lines',
            name: `ADC ${i + 1}`,
            line: { color: colors[i] }
        });
    }

    const layout = {
        title: 'ADC Channels Real-time Data',
        xaxis: {
            title: 'Time (s)',
            showgrid: true,
            zeroline: false
        },
        yaxis: {
            title: 'ADC Value',
            showgrid: true,
            zeroline: false
        },
        showlegend: true,
        legend: {
            x: 1,
            xanchor: 'right',
            y: 1
        }
    };

    Plotly.newPlot('chart', traces, layout);
    chart = document.getElementById('chart');
}

// Fetch data from the server and update the chart
async function updateChart() {
    try {
        const response = await fetch('/api/data');
        const data = await response.json();

        // Update each trace with new data
        const update = {
            x: [],
            y: []
        };

        for (let i = 0; i < data.values.length; i++) {
            update.x.push([data.timestamps]);
            update.y.push([data.values[i]]);
        }

        Plotly.update('chart', update);
    } catch (error) {
        console.error('Error fetching data:', error);
    }
}

// Event handlers for start/stop buttons
document.getElementById('startBtn').addEventListener('click', () => {
    if (!updateInterval) {
        updateInterval = setInterval(updateChart, UPDATE_INTERVAL_MS);
        document.getElementById('startBtn').disabled = true;
        document.getElementById('stopBtn').disabled = false;
    }
});

document.getElementById('stopBtn').addEventListener('click', () => {
    if (updateInterval) {
        clearInterval(updateInterval);
        updateInterval = null;
        document.getElementById('startBtn').disabled = false;
        document.getElementById('stopBtn').disabled = true;
    }
});

// Initialize the chart when the page loads
window.addEventListener('load', () => {
    initChart();
    document.getElementById('stopBtn').disabled = true;
});

