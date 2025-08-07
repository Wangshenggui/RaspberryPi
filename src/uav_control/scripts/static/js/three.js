// 创建场景
const scene = new THREE.Scene();

// 创建摄像机
const camera = new THREE.PerspectiveCamera(
  75, window.innerWidth / window.innerHeight, 0.1, 1000
);
camera.position.set(5, 5, 5); // 斜着看
camera.lookAt(0, 0, 0); // 目标是原点

// 创建渲染器
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// 半透明材质，分别为不同的面设置不同的颜色
const planeMaterialRed = new THREE.MeshBasicMaterial({
  color: 0xff0000,  // 红色
  transparent: true,
  opacity: 0.3,
  side: THREE.DoubleSide
});

const planeMaterialGreen = new THREE.MeshBasicMaterial({
  color: 0x00ff00,  // 绿色
  transparent: true,
  opacity: 0.3,
  side: THREE.DoubleSide
});

const planeMaterialBlue = new THREE.MeshBasicMaterial({
  color: 0x0000ff,  // 蓝色
  transparent: true,
  opacity: 0.3,
  side: THREE.DoubleSide
});

// XY 平面（红色）
const xyPlaneGeometry = new THREE.PlaneGeometry(5, 5);
const xyPlane = new THREE.Mesh(xyPlaneGeometry, planeMaterialRed);
xyPlane.rotation.x = Math.PI / 2;
xyPlane.position.set(0, 0, 0);
scene.add(xyPlane);

// XZ 平面（绿色）
const xzPlaneGeometry = new THREE.PlaneGeometry(5, 5);
const xzPlane = new THREE.Mesh(xzPlaneGeometry, planeMaterialGreen);
xzPlane.rotation.y = Math.PI / 2;
xzPlane.position.set(0, 0, 0);
scene.add(xzPlane);

// YZ 平面（蓝色）
const yzPlaneGeometry = new THREE.PlaneGeometry(5, 5);
const yzPlane = new THREE.Mesh(yzPlaneGeometry, planeMaterialBlue);
yzPlane.rotation.z = Math.PI / 2;
yzPlane.position.set(0, 0, 0);
scene.add(yzPlane);

// 创建小红点（用于演示）
const pointGeometry = new THREE.SphereGeometry(0.05, 16, 16);
const pointMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
const point = new THREE.Mesh(pointGeometry, pointMaterial);
point.position.set(0, 0, 1);
scene.add(point);

// 添加坐标轴辅助器
const axesHelper = new THREE.AxesHelper(5); // 坐标轴更大
scene.add(axesHelper);

// 动画循环
let t = 0;
function animate() {
  requestAnimationFrame(animate);

  // 动态控制点的浮动，确保更新到最新的全局变量
  point.position.set(window.x, window.y, window.z);  // 点沿Z轴浮动
  t += 0.02;

  renderer.render(scene, camera);
}

animate();

// 自适应窗口大小
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// 设置默认的缩放比例
let zoomFactor = 1;

// 添加缩放事件监听器
window.addEventListener('wheel', (event) => {
  if (event.deltaY < 0) {
    zoomFactor *= 1.1;  // 向上滚动，缩小视图
  } else {
    zoomFactor /= 1.1;  // 向下滚动，放大视图
  }

  // 更新摄像机的 zoom 属性
  camera.zoom = zoomFactor;
  camera.updateProjectionMatrix();
});
