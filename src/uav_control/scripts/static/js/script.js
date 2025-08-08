// 获取元素
const speedPValue = document.getElementById('speed_p_value');
const speedIValue = document.getElementById('speed_i_value');
const speedDValue = document.getElementById('speed_d_value');
const positionPValue = document.getElementById('position_p_value');
const positionIValue = document.getElementById('position_i_value');
const positionDValue = document.getElementById('position_d_value');

// 获取按钮
const speedPIncrease1 = document.getElementById('speed_p_increase_1');
const speedPDecrease1 = document.getElementById('speed_p_decrease_1');
const speedPIncrease0_1 = document.getElementById('speed_p_increase_0_1');
const speedPDecrease0_1 = document.getElementById('speed_p_decrease_0_1');
const speedPIncrease0_01 = document.getElementById('speed_p_increase_0_01');
const speedPDecrease0_01 = document.getElementById('speed_p_decrease_0_01');

const speedIIncrease1 = document.getElementById('speed_i_increase_1');
const speedIDecrease1 = document.getElementById('speed_i_decrease_1');
const speedIIncrease0_1 = document.getElementById('speed_i_increase_0_1');
const speedIDecrease0_1 = document.getElementById('speed_i_decrease_0_1');
const speedIIncrease0_01 = document.getElementById('speed_i_increase_0_01');
const speedIDecrease0_01 = document.getElementById('speed_i_decrease_0_01');

const speedDIncrease1 = document.getElementById('speed_d_increase_1');
const speedDDecrease1 = document.getElementById('speed_d_decrease_1');
const speedDIncrease0_1 = document.getElementById('speed_d_increase_0_1');
const speedDDecrease0_1 = document.getElementById('speed_d_decrease_0_1');
const speedDIncrease0_01 = document.getElementById('speed_d_increase_0_01');
const speedDDecrease0_01 = document.getElementById('speed_d_decrease_0_01');

const positionPIncrease1 = document.getElementById('position_p_increase_1');
const positionPDecrease1 = document.getElementById('position_p_decrease_1');
const positionPIncrease0_1 = document.getElementById('position_p_increase_0_1');
const positionPDecrease0_1 = document.getElementById('position_p_decrease_0_1');
const positionPIncrease0_01 = document.getElementById('position_p_increase_0_01');
const positionPDecrease0_01 = document.getElementById('position_p_decrease_0_01');

const positionIIncrease1 = document.getElementById('position_i_increase_1');
const positionIDecrease1 = document.getElementById('position_i_decrease_1');
const positionIIncrease0_1 = document.getElementById('position_i_increase_0_1');
const positionIDecrease0_1 = document.getElementById('position_i_decrease_0_1');
const positionIIncrease0_01 = document.getElementById('position_i_increase_0_01');
const positionIDecrease0_01 = document.getElementById('position_i_decrease_0_01');

const positionDIncrease1 = document.getElementById('position_d_increase_1');
const positionDDecrease1 = document.getElementById('position_d_decrease_1');
const positionDIncrease0_1 = document.getElementById('position_d_increase_0_1');
const positionDDecrease0_1 = document.getElementById('position_d_decrease_0_1');
const positionDIncrease0_01 = document.getElementById('position_d_increase_0_01');
const positionDDecrease0_01 = document.getElementById('position_d_decrease_0_01');

// 获取复位按钮
const speedPReset = document.getElementById('speed_p_reset');
const speedIReset = document.getElementById('speed_i_reset');
const speedDReset = document.getElementById('speed_d_reset');

const positionPReset = document.getElementById('position_p_reset');
const positionIReset = document.getElementById('position_i_reset');
const positionDReset = document.getElementById('position_d_reset');

// 重新建图
const hector_slam = document.getElementById('hector_slam');

// 复位函数
function resetValue(inputElement) {
    inputElement.value = "0";
    updateAndSendData();
}

// 添加复位按钮事件
speedPReset.addEventListener('click', () => resetValue(speedPValue));
speedIReset.addEventListener('click', () => resetValue(speedIValue));
speedDReset.addEventListener('click', () => resetValue(speedDValue));
positionPReset.addEventListener('click', () => resetValue(positionPValue));
positionIReset.addEventListener('click', () => resetValue(positionIValue));
positionDReset.addEventListener('click', () => resetValue(positionDValue));

// 设置步长
const steps = {
    speed: [1, 0.1, 0.01],  // 速度环的 P, I, D 参数步长
    position: [1, 0.1, 0.01]  // 位置环的 P, I, D 参数步长
};

// 创建一个函数，用于每次调整后立即发送数据
async function updateAndSendData() {
    const pidData = {
        speed_p: parseFloat(speedPValue.value),
        speed_i: parseFloat(speedIValue.value),
        speed_d: parseFloat(speedDValue.value),
        position_p: parseFloat(positionPValue.value),
        position_i: parseFloat(positionIValue.value),
        position_d: parseFloat(positionDValue.value)
    };

    const response = await fetch('/client_to_server', {
        method: 'POST',
        headers: {
            'Content-Type': 'text/plain'
        },
        body: JSON.stringify(pidData)
    });

    // const result = await response.json();  // 解析返回的 JSON 数据
    // if (response.ok) {
    //     // 显示从后端返回的原始数据
    //     document.getElementById('data-display').textContent = JSON.stringify(result, null, 2);
    // } else {
    //     // 显示错误信息
    //     document.getElementById('data-display').textContent = `错误: ${result.error}`;
    // }
}

// 获取 canvas 元素
const canvas = document.getElementById('trajectoryCanvas');
const ctx1 = canvas.getContext('2d');

// 记录上次的坐标
let lastX = 0;
let lastY = 0;

// 计算画布中心点
const centerX = canvas.width / 2;
const centerY = canvas.height / 2;

// 定义绘制轨迹的函数
// 用数组保存轨迹点
let points = [];
const maxPoints = 20;

function drawTrajectory(x, y) {
    let canvasX = centerX - x;
    let canvasY = centerY - y;

    // 加入新点
    points.push({ x: canvasX, y: canvasY });

    // 如果超出最大数量，删除最前面的点
    if (points.length > maxPoints) {
        points.shift();
    }

    // 清空画布（不清就会重叠）
    ctx1.clearRect(0, 0, canvas.width, canvas.height);

    // 画中心点
    ctx1.beginPath();
    ctx1.arc(centerX, centerY, 1, 0, Math.PI * 2, false);
    ctx1.fillStyle = 'red';
    ctx1.fill();

    // 画轨迹点
    ctx1.fillStyle = 'blue';
    points.forEach(p => {
        ctx1.beginPath();
        ctx1.arc(p.x, p.y, 3, 0, Math.PI * 2, false);
        ctx1.fill();
    });
}

const dataPoints = 300;  // 图表数据点的数量

// 生成随机数的函数
function generateRandomData() {
    return Math.floor(Math.random() * 201) - 100; // 生成一个 -100 到 100 之间的随机数
}

// 生成时间标签
let timeLabels = [];
for (let i = 0; i < dataPoints; i++) {
    timeLabels.push(i);
}

// 初始化数据
let randomData = Array(dataPoints).fill(0).map(generateRandomData);

// 获取 canvas 元素
var ctx2 = document.getElementById('myChart').getContext('2d');

// 创建图表
var myChart = new Chart(ctx2, {
    type: 'line',  // 图表类型：线性图
    data: {
        labels: timeLabels,  // 时间数据
        datasets: [{
            label: 'Random Y-T Curve',  // 曲线的名称
            data: randomData,  // 随机生成的 Y 数据
            fill: false,  // 不填充曲线下方区域
            borderColor: 'rgb(75, 192, 192)',  // 曲线颜色
            tension: 1  // 曲线的弯曲程度
        }]
    },
    options: {
        responsive: true,  // 启用响应式
        scales: {
            x: {
                title: {
                    display: true,
                    text: 'Time (T)'
                }
            },
            y: {
                min: -50,  // Y轴最小值
                max: 50, // Y轴最大值（根据你的数据范围调整）
                beginAtZero: false,
                ticks: {
                    // 关闭自动缩放行为
                    autoSkip: false,
                    callback: function(value) {
                        return value; // 可以自定义显示格式
                    }
                },
                // 关键参数：一定要加这个，避免 scale 自动调整！
                suggestedMin: 0,
                suggestedMax: 100,
                title: {
                    display: true,
                    text: 'Value (Y)'
                }
            }
        }
    }
});

// 定义变量为全局变量
window.x = 1;
window.y = 2;
window.z = 3;
// 每隔 2 秒请求一次后端
setInterval(async function() {
    const response = await fetch('/server_to_client');  // 向后端请求数据
    const result = await response.json();  // 解析为 JavaScript 对象

    // 显示多个值在一个元素中
    document.getElementById('data-display').textContent = 
        `x-Outer: ${result["x-Outer"]}, x-Inner: ${result["x-Inner"]}, ` +
        `y-Outer: ${result["y-Outer"]}, y-Inner: ${result["y-Inner"]}, ` + 
        `z-Pose: ${result["z-Pose"].toFixed(1)}, ` + 
        `z-out: ${result["z-out"]}`;

    speedPValue.value = result["kp_velocity"];
    speedIValue.value = result["ki_velocity"];
    speedDValue.value = result["kd_velocity"];

    positionPValue.value = result["kp_position"];
    positionIValue.value = result["ki_position"];
    positionDValue.value = result["kd_position"];

    // 开始绘制轨迹
    drawTrajectory(result["y-Pose"]*200, result["x-Pose"]*200);
    window.x = result["y-Pose"]*100;
    window.x = result["x-Pose"]*100;

    
    // 删除最左侧的数据点
    randomData.shift();

    // 生成一个新的随机数据点并添加到数据数组的右侧
    randomData.push(result["x-Outer"]);

    // 时间标签也要左移，新增一个标签
    timeLabels.shift();
    timeLabels.push(timeLabels[timeLabels.length - 1] + 1);

    // 更新图表的数据
    myChart.data.labels = timeLabels;
    myChart.data.datasets[0].data = randomData;

    // 更新图表
    myChart.update();
}, 100);  // 每隔1秒请求一次数据

// 调整速度环的 PID 参数
speedPIncrease1.addEventListener('click', () => {
    speedPValue.value = (parseFloat(speedPValue.value) + steps.speed[0]).toFixed(0);
    updateAndSendData();
});

speedPDecrease1.addEventListener('click', () => {
    speedPValue.value = (parseFloat(speedPValue.value) - steps.speed[0]).toFixed(0);
    updateAndSendData();
});

speedPIncrease0_1.addEventListener('click', () => {
    speedPValue.value = (parseFloat(speedPValue.value) + steps.speed[1]).toFixed(1);
    updateAndSendData();
});

speedPDecrease0_1.addEventListener('click', () => {
    speedPValue.value = (parseFloat(speedPValue.value) - steps.speed[1]).toFixed(1);
    updateAndSendData();
});

speedPIncrease0_01.addEventListener('click', () => {
    speedPValue.value = (parseFloat(speedPValue.value) + steps.speed[2]).toFixed(2);
    updateAndSendData();
});

speedPDecrease0_01.addEventListener('click', () => {
    speedPValue.value = (parseFloat(speedPValue.value) - steps.speed[2]).toFixed(2);
    updateAndSendData();
});

speedIIncrease1.addEventListener('click', () => {
    speedIValue.value = (parseFloat(speedIValue.value) + steps.speed[0]).toFixed(0);
    updateAndSendData();
});

speedIDecrease1.addEventListener('click', () => {
    speedIValue.value = (parseFloat(speedIValue.value) - steps.speed[0]).toFixed(0);
    updateAndSendData();
});

speedIIncrease0_1.addEventListener('click', () => {
    speedIValue.value = (parseFloat(speedIValue.value) + steps.speed[1]).toFixed(1);
    updateAndSendData();
});

speedIDecrease0_1.addEventListener('click', () => {
    speedIValue.value = (parseFloat(speedIValue.value) - steps.speed[1]).toFixed(1);
    updateAndSendData();
});

speedIIncrease0_01.addEventListener('click', () => {
    speedIValue.value = (parseFloat(speedIValue.value) + steps.speed[2]).toFixed(2);
    updateAndSendData();
});

speedIDecrease0_01.addEventListener('click', () => {
    speedIValue.value = (parseFloat(speedIValue.value) - steps.speed[2]).toFixed(2);
    updateAndSendData();
});

speedDIncrease1.addEventListener('click', () => {
    speedDValue.value = (parseFloat(speedDValue.value) + steps.speed[0]).toFixed(0);
    updateAndSendData();
});

speedDDecrease1.addEventListener('click', () => {
    speedDValue.value = (parseFloat(speedDValue.value) - steps.speed[0]).toFixed(0);
    updateAndSendData();
});

speedDIncrease0_1.addEventListener('click', () => {
    speedDValue.value = (parseFloat(speedDValue.value) + steps.speed[1]).toFixed(1);
    updateAndSendData();
});

speedDDecrease0_1.addEventListener('click', () => {
    speedDValue.value = (parseFloat(speedDValue.value) - steps.speed[1]).toFixed(1);
    updateAndSendData();
});

speedDIncrease0_01.addEventListener('click', () => {
    speedDValue.value = (parseFloat(speedDValue.value) + steps.speed[2]).toFixed(2);
    updateAndSendData();
});

speedDDecrease0_01.addEventListener('click', () => {
    speedDValue.value = (parseFloat(speedDValue.value) - steps.speed[2]).toFixed(2);
    updateAndSendData();
});

// 调整位置环的 PID 参数
positionPIncrease1.addEventListener('click', () => {
    positionPValue.value = (parseFloat(positionPValue.value) + steps.position[0]).toFixed(0);
    updateAndSendData();
});

positionPDecrease1.addEventListener('click', () => {
    positionPValue.value = (parseFloat(positionPValue.value) - steps.position[0]).toFixed(0);
    updateAndSendData();
});

positionPIncrease0_1.addEventListener('click', () => {
    positionPValue.value = (parseFloat(positionPValue.value) + steps.position[1]).toFixed(1);
    updateAndSendData();
});

positionPDecrease0_1.addEventListener('click', () => {
    positionPValue.value = (parseFloat(positionPValue.value) - steps.position[1]).toFixed(1);
    updateAndSendData();
});

positionPIncrease0_01.addEventListener('click', () => {
    positionPValue.value = (parseFloat(positionPValue.value) + steps.position[2]).toFixed(2);
    updateAndSendData();
});

positionPDecrease0_01.addEventListener('click', () => {
    positionPValue.value = (parseFloat(positionPValue.value) - steps.position[2]).toFixed(2);
    updateAndSendData();
});

positionIIncrease1.addEventListener('click', () => {
    positionIValue.value = (parseFloat(positionIValue.value) + steps.position[0]).toFixed(0);
    updateAndSendData();
});

positionIDecrease1.addEventListener('click', () => {
    positionIValue.value = (parseFloat(positionIValue.value) - steps.position[0]).toFixed(0);
    updateAndSendData();
});

positionIIncrease0_1.addEventListener('click', () => {
    positionIValue.value = (parseFloat(positionIValue.value) + steps.position[1]).toFixed(1);
    updateAndSendData();
});

positionIDecrease0_1.addEventListener('click', () => {
    positionIValue.value = (parseFloat(positionIValue.value) - steps.position[1]).toFixed(1);
    updateAndSendData();
});

positionIIncrease0_01.addEventListener('click', () => {
    positionIValue.value = (parseFloat(positionIValue.value) + steps.position[2]).toFixed(2);
    updateAndSendData();
});

positionIDecrease0_01.addEventListener('click', () => {
    positionIValue.value = (parseFloat(positionIValue.value) - steps.position[2]).toFixed(2);
    updateAndSendData();
});

positionDIncrease1.addEventListener('click', () => {
    positionDValue.value = (parseFloat(positionDValue.value) + steps.position[0]).toFixed(0);
    updateAndSendData();
});

positionDDecrease1.addEventListener('click', () => {
    positionDValue.value = (parseFloat(positionDValue.value) - steps.position[0]).toFixed(0);
    updateAndSendData();
});

positionDIncrease0_1.addEventListener('click', () => {
    positionDValue.value = (parseFloat(positionDValue.value) + steps.position[1]).toFixed(1);
    updateAndSendData();
});

positionDDecrease0_1.addEventListener('click', () => {
    positionDValue.value = (parseFloat(positionDValue.value) - steps.position[1]).toFixed(1);
    updateAndSendData();
});

positionDIncrease0_01.addEventListener('click', () => {
    positionDValue.value = (parseFloat(positionDValue.value) + steps.position[2]).toFixed(2);
    updateAndSendData();
});

positionDDecrease0_01.addEventListener('click', () => {
    positionDValue.value = (parseFloat(positionDValue.value) - steps.position[2]).toFixed(2);
    updateAndSendData();
});

// 创建一个函数，用于每次调整后立即发送数据
async function hector_slam_updateAndSendData() {
    // 第一次发送，带 hector_slam: "true"
    const pidDataWithFlag = {
        speed_p: parseFloat(speedPValue.value),
        speed_i: parseFloat(speedIValue.value),
        speed_d: parseFloat(speedDValue.value),
        position_p: parseFloat(positionPValue.value),
        position_i: parseFloat(positionIValue.value),
        position_d: parseFloat(positionDValue.value),
        hector_slam: "true"
    };

    await fetch('/client_to_server', {
        method: 'POST',
        headers: {
            'Content-Type': 'text/plain'
        },
        body: JSON.stringify(pidDataWithFlag)
    });
    await new Promise(resolve => setTimeout(resolve, 200));

    // 第二次发送，移除 hector_slam 字段
    const pidDataNoFlag = {
        speed_p: pidDataWithFlag.speed_p,
        speed_i: pidDataWithFlag.speed_i,
        speed_d: pidDataWithFlag.speed_d,
        position_p: pidDataWithFlag.position_p,
        position_i: pidDataWithFlag.position_i,
        position_d: pidDataWithFlag.position_d
    };

    await fetch('/client_to_server', {
        method: 'POST',
        headers: {
            'Content-Type': 'text/plain'
        },
        body: JSON.stringify(pidDataNoFlag)
    });
}
// 重新建图
hector_slam.addEventListener('click', () => {
    hector_slam_updateAndSendData();
});