import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js';

const canvas = document.getElementById('viewerCanvas');
const folderInput = document.getElementById('folderInput');
const openFolderButton = document.getElementById('openFolderButton');
const urdfSelect = document.getElementById('urdfSelect');
const folderName = document.getElementById('folderName');
const robotName = document.getElementById('robotName');
const activeFile = document.getElementById('activeFile');
const statusText = document.getElementById('statusText');
const issueList = document.getElementById('issueList');
const linkCount = document.getElementById('linkCount');
const jointCount = document.getElementById('jointCount');
const meshCount = document.getElementById('meshCount');
const fitButton = document.getElementById('fitButton');
const clearButton = document.getElementById('clearButton');
const meshToggle = document.getElementById('meshToggle');
const wireToggle = document.getElementById('wireToggle');
const gridToggle = document.getElementById('gridToggle');
const axesToggle = document.getElementById('axesToggle');

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x111214, 1);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 2000);
const robotRoot = new THREE.Group();
robotRoot.rotation.x = -Math.PI / 2;
scene.add(robotRoot);

const grid = new THREE.GridHelper(10, 20, 0x53605a, 0x2f3432);
scene.add(grid);

const axes = new THREE.AxesHelper(0.45);
scene.add(axes);

scene.add(new THREE.HemisphereLight(0xf5f7f4, 0x2b2f2c, 2.6));
const keyLight = new THREE.DirectionalLight(0xffffff, 2.2);
keyLight.position.set(3, 5, 4);
scene.add(keyLight);
const fillLight = new THREE.DirectionalLight(0xeab75a, 0.7);
fillLight.position.set(-4, 2, -3);
scene.add(fillLight);

let indexedFiles = new Map();
let urdfOptions = [];
let meshObjects = [];
let currentIssues = [];
let currentPackageRoot = '';

const orbit = {
  target: new THREE.Vector3(0, 0, 0),
  radius: 3,
  theta: Math.PI / 4,
  phi: Math.PI / 3,
  dragging: false,
  button: 0,
  lastX: 0,
  lastY: 0
};

function setStatus(message, issues = []) {
  statusText.textContent = message;
  currentIssues = issues;
  issueList.replaceChildren(...issues.slice(0, 12).map((issue) => {
    const item = document.createElement('li');
    item.textContent = issue;
    return item;
  }));
}

function resetMetrics() {
  linkCount.textContent = '0';
  jointCount.textContent = '0';
  meshCount.textContent = '0';
  robotName.textContent = 'Sin modelo';
  activeFile.textContent = 'Sin archivo';
}

function normalizePath(path) {
  return path.replace(/\\/g, '/').replace(/^\.\/+/, '').replace(/\/+/g, '/');
}

function stripFirstSegment(path) {
  const parts = normalizePath(path).split('/');
  return parts.length > 1 ? parts.slice(1).join('/') : path;
}

function basename(path) {
  return normalizePath(path).split('/').pop();
}

function indexFolderFiles(fileList) {
  indexedFiles = new Map();
  urdfOptions = [];
  currentPackageRoot = '';

  for (const file of fileList) {
    const fullPath = normalizePath(file.webkitRelativePath || file.name);
    const lowerFullPath = fullPath.toLowerCase();
    const strippedPath = stripFirstSegment(fullPath);
    const lowerStrippedPath = strippedPath.toLowerCase();

    indexedFiles.set(lowerFullPath, file);
    indexedFiles.set(lowerStrippedPath, file);
    indexedFiles.set(basename(fullPath).toLowerCase(), file);

    if (!currentPackageRoot && fullPath.includes('/')) {
      currentPackageRoot = fullPath.split('/')[0];
    }

    if (isRobotDescriptionFile(fullPath) && (/(^|\/)urdf\//i.test(fullPath) || fullPath.split('/').length <= 2)) {
      urdfOptions.push({ path: fullPath, file });
    }
  }

  urdfOptions.sort((a, b) => {
    const aScore = a.path.toLowerCase().endsWith('.urdf') ? 0 : 1;
    const bScore = b.path.toLowerCase().endsWith('.urdf') ? 0 : 1;
    return aScore - bScore || a.path.localeCompare(b.path);
  });
}

function isRobotDescriptionFile(path) {
  const lowerPath = path.toLowerCase();
  if (lowerPath.endsWith('.urdf')) return true;
  if (!lowerPath.endsWith('.xacro')) return false;
  return !lowerPath.endsWith('materials.xacro')
    && !lowerPath.endsWith('.gazebo.xacro')
    && !lowerPath.endsWith('.ros2_control.xacro');
}

function populateUrdfSelect() {
  urdfSelect.replaceChildren();
  urdfSelect.disabled = urdfOptions.length === 0;

  if (urdfOptions.length === 0) {
    const option = document.createElement('option');
    option.textContent = 'No se encontro URDF';
    urdfSelect.appendChild(option);
    return;
  }

  for (const optionInfo of urdfOptions) {
    const option = document.createElement('option');
    option.value = optionInfo.path;
    option.textContent = optionInfo.path;
    urdfSelect.appendChild(option);
  }
}

function clearRobot() {
  for (const mesh of meshObjects) {
    mesh.geometry?.dispose();
    if (Array.isArray(mesh.material)) {
      mesh.material.forEach((material) => material.dispose());
    } else {
      mesh.material?.dispose();
    }
  }
  meshObjects = [];
  robotRoot.clear();
  resetMetrics();
}

function directChildren(element, tagName) {
  return Array.from(element.children).filter((child) => child.localName === tagName || child.tagName === tagName);
}

function firstDirectChild(element, tagName) {
  return directChildren(element, tagName)[0] || null;
}

function parseVector(value, fallback = [0, 0, 0]) {
  if (!value) return fallback.slice();
  const parts = value.trim().split(/\s+/).map(Number);
  return parts.length >= 3 && parts.every(Number.isFinite) ? parts.slice(0, 3) : fallback.slice();
}

function parseOrigin(element) {
  const origin = firstDirectChild(element, 'origin');
  return {
    xyz: parseVector(origin?.getAttribute('xyz'), [0, 0, 0]),
    rpy: parseVector(origin?.getAttribute('rpy'), [0, 0, 0])
  };
}

function applyOrigin(object, origin) {
  object.position.set(origin.xyz[0], origin.xyz[1], origin.xyz[2]);
  object.rotation.set(origin.rpy[0], origin.rpy[1], origin.rpy[2], 'XYZ');
}

function parseMaterialColor(materialElement, materialMap) {
  if (!materialElement) return null;
  const inlineColor = firstDirectChild(materialElement, 'color')?.getAttribute('rgba');
  const namedColor = materialMap.get(materialElement.getAttribute('name') || '');
  const rgba = inlineColor || namedColor;
  if (!rgba) return null;
  const values = rgba.trim().split(/\s+/).map(Number);
  if (values.length < 3 || !values.slice(0, 3).every(Number.isFinite)) return null;
  return {
    color: new THREE.Color(values[0], values[1], values[2]),
    opacity: Number.isFinite(values[3]) ? values[3] : 1
  };
}

function parseUrdfDocument(text) {
  const doc = new DOMParser().parseFromString(text, 'application/xml');
  const parserError = doc.querySelector('parsererror');
  if (parserError) {
    throw new Error(parserError.textContent.trim().split('\n')[0]);
  }
  if (doc.documentElement.localName !== 'robot') {
    throw new Error('El XML no contiene un elemento robot.');
  }
  return doc;
}

function collectRobotData(doc) {
  const robot = doc.documentElement;
  const materials = new Map();
  for (const material of directChildren(robot, 'material')) {
    const name = material.getAttribute('name');
    const color = firstDirectChild(material, 'color')?.getAttribute('rgba');
    if (name && color) materials.set(name, color);
  }

  const links = new Map();
  for (const link of directChildren(robot, 'link')) {
    const name = link.getAttribute('name');
    if (!name) continue;
    const visuals = directChildren(link, 'visual').map((visual) => {
      const geometry = firstDirectChild(visual, 'geometry');
      const material = firstDirectChild(visual, 'material');
      return {
        origin: parseOrigin(visual),
        geometry,
        materialColor: parseMaterialColor(material, materials)
      };
    });
    links.set(name, { name, visuals, joints: [] });
  }

  const joints = [];
  for (const joint of directChildren(robot, 'joint')) {
    const parent = firstDirectChild(joint, 'parent')?.getAttribute('link');
    const child = firstDirectChild(joint, 'child')?.getAttribute('link');
    const name = joint.getAttribute('name') || `${parent || 'link'}_${child || 'link'}`;
    if (!parent || !child) continue;
    joints.push({ name, parent, child, origin: parseOrigin(joint), type: joint.getAttribute('type') || 'fixed' });
    if (links.has(parent)) {
      links.get(parent).joints.push(joints[joints.length - 1]);
    }
  }

  return { name: robot.getAttribute('name') || 'robot', links, joints };
}

function resolveMeshFile(filename) {
  if (!filename) return null;
  let clean = normalizePath(filename.trim());
  clean = clean.replace(/^file:\/+/, '');

  const candidates = [];
  if (clean.startsWith('package://')) {
    const withoutScheme = clean.replace(/^package:\/\//, '');
    candidates.push(withoutScheme);
    const parts = withoutScheme.split('/');
    if (parts.length > 1) candidates.push(parts.slice(1).join('/'));
  } else {
    candidates.push(clean);
  }

  candidates.push(stripFirstSegment(clean));
  candidates.push(`meshes/${basename(clean)}`);
  candidates.push(basename(clean));

  for (const candidate of candidates) {
    const file = indexedFiles.get(normalizePath(candidate).toLowerCase());
    if (file) return file;
  }

  return null;
}

function makeMaterial(materialColor) {
  const color = materialColor?.color || new THREE.Color(0.72, 0.74, 0.70);
  const opacity = materialColor?.opacity ?? 1;
  return new THREE.MeshStandardMaterial({
    color,
    roughness: 0.76,
    metalness: 0.08,
    transparent: opacity < 1,
    opacity,
    side: THREE.DoubleSide,
    wireframe: wireToggle.checked
  });
}

function geometryFromPrimitive(geometryElement) {
  if (!geometryElement) return null;
  const box = firstDirectChild(geometryElement, 'box');
  const cylinder = firstDirectChild(geometryElement, 'cylinder');
  const sphere = firstDirectChild(geometryElement, 'sphere');

  if (box) {
    const size = parseVector(box.getAttribute('size'), [1, 1, 1]);
    return new THREE.BoxGeometry(size[0], size[1], size[2]);
  }

  if (cylinder) {
    const radius = Number(cylinder.getAttribute('radius')) || 0.5;
    const length = Number(cylinder.getAttribute('length')) || 1;
    const geometry = new THREE.CylinderGeometry(radius, radius, length, 36);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }

  if (sphere) {
    const radius = Number(sphere.getAttribute('radius')) || 0.5;
    return new THREE.SphereGeometry(radius, 36, 18);
  }

  return null;
}

async function loadVisualMesh(visual, linkName, issues) {
  const visualGroup = new THREE.Group();
  applyOrigin(visualGroup, visual.origin);

  const meshElement = visual.geometry ? visual.geometry.getElementsByTagName('mesh')[0] : null;
  const material = makeMaterial(visual.materialColor);
  let geometry = null;

  if (meshElement) {
    const filename = meshElement.getAttribute('filename');
    const meshFile = resolveMeshFile(filename);
    if (!meshFile) {
      issues.push(`No encontre la malla de ${linkName}: ${filename}`);
      material.dispose();
      return visualGroup;
    }
    geometry = await readStlFile(meshFile);
    const scale = parseVector(meshElement.getAttribute('scale'), [1, 1, 1]);
    visualGroup.scale.set(scale[0], scale[1], scale[2]);
  } else {
    geometry = geometryFromPrimitive(visual.geometry);
  }

  if (!geometry) {
    material.dispose();
    issues.push(`Visual sin geometria soportada en ${linkName}`);
    return visualGroup;
  }

  geometry.computeBoundingSphere();
  geometry.computeBoundingBox();
  const mesh = new THREE.Mesh(geometry, material);
  mesh.name = linkName;
  meshObjects.push(mesh);
  visualGroup.add(mesh);
  return visualGroup;
}

async function readStlFile(file) {
  const buffer = await file.arrayBuffer();
  return parseStl(buffer);
}

function parseStl(buffer) {
  const dataView = new DataView(buffer);
  const triangleCount = buffer.byteLength >= 84 ? dataView.getUint32(80, true) : 0;
  const expectedBinaryLength = 84 + triangleCount * 50;

  if (triangleCount > 0 && expectedBinaryLength === buffer.byteLength) {
    return parseBinaryStl(dataView, triangleCount);
  }

  const text = new TextDecoder().decode(buffer);
  return parseAsciiStl(text);
}

function parseBinaryStl(dataView, triangleCount) {
  const positions = new Float32Array(triangleCount * 9);
  const normals = new Float32Array(triangleCount * 9);
  let offset = 84;
  let positionIndex = 0;

  for (let face = 0; face < triangleCount; face += 1) {
    const normal = [
      dataView.getFloat32(offset, true),
      dataView.getFloat32(offset + 4, true),
      dataView.getFloat32(offset + 8, true)
    ];
    offset += 12;

    for (let vertex = 0; vertex < 3; vertex += 1) {
      positions[positionIndex] = dataView.getFloat32(offset, true);
      positions[positionIndex + 1] = dataView.getFloat32(offset + 4, true);
      positions[positionIndex + 2] = dataView.getFloat32(offset + 8, true);
      normals[positionIndex] = normal[0];
      normals[positionIndex + 1] = normal[1];
      normals[positionIndex + 2] = normal[2];
      offset += 12;
      positionIndex += 3;
    }
    offset += 2;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
  return geometry;
}

function parseAsciiStl(text) {
  const vertices = [];
  const regex = /vertex\s+([+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?)\s+([+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?)\s+([+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?)/gi;
  let match = regex.exec(text);
  while (match) {
    vertices.push(Number(match[1]), Number(match[2]), Number(match[3]));
    match = regex.exec(text);
  }

  if (vertices.length === 0) {
    throw new Error('STL ASCII sin vertices.');
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(vertices), 3));
  geometry.computeVertexNormals();
  return geometry;
}

async function buildRobot(data) {
  clearRobot();

  const groups = new Map();
  const promises = [];
  const issues = [];
  const childLinks = new Set(data.joints.map((joint) => joint.child));
  const rootName = data.links.has('base_link')
    ? 'base_link'
    : Array.from(data.links.keys()).find((name) => !childLinks.has(name)) || Array.from(data.links.keys())[0];

  function ensureGroup(linkName) {
    if (groups.has(linkName)) return groups.get(linkName);

    const group = new THREE.Group();
    group.name = linkName;
    groups.set(linkName, group);

    const link = data.links.get(linkName);
    if (link) {
      for (const visual of link.visuals) {
        promises.push(loadVisualMesh(visual, linkName, issues).then((visualGroup) => group.add(visualGroup)).catch((error) => {
          issues.push(`${linkName}: ${error.message}`);
        }));
      }
    }

    return group;
  }

  function attachTree(linkName, parentGroup) {
    const linkGroup = ensureGroup(linkName);
    if (!linkGroup.parent) parentGroup.add(linkGroup);

    const link = data.links.get(linkName);
    if (!link) return;

    for (const joint of link.joints) {
      const childGroup = ensureGroup(joint.child);
      applyOrigin(childGroup, joint.origin);
      linkGroup.add(childGroup);
      attachTree(joint.child, childGroup);
    }
  }

  if (rootName) {
    attachTree(rootName, robotRoot);
  }

  for (const linkName of data.links.keys()) {
    const group = ensureGroup(linkName);
    if (!group.parent) robotRoot.add(group);
  }

  await Promise.allSettled(promises);

  linkCount.textContent = String(data.links.size);
  jointCount.textContent = String(data.joints.length);
  meshCount.textContent = String(meshObjects.length);
  robotName.textContent = data.name;
  applyVisibilityState();
  fitCameraToRobot();

  const message = issues.length
    ? `Modelo cargado con ${issues.length} aviso(s).`
    : 'Modelo cargado.';
  setStatus(message, issues);
}

async function loadSelectedUrdf() {
  const selectedPath = urdfSelect.value;
  const selected = urdfOptions.find((option) => option.path === selectedPath);
  if (!selected) return;

  setStatus('Cargando modelo...');
  try {
    const text = await selected.file.text();
    const doc = parseUrdfDocument(text);
    const data = collectRobotData(doc);
    activeFile.textContent = selected.path;
    await buildRobot(data);
  } catch (error) {
    clearRobot();
    setStatus('No pude cargar el URDF.', [error.message]);
  }
}

function applyVisibilityState() {
  for (const mesh of meshObjects) {
    mesh.visible = meshToggle.checked;
    if (Array.isArray(mesh.material)) {
      mesh.material.forEach((material) => { material.wireframe = wireToggle.checked; });
    } else {
      mesh.material.wireframe = wireToggle.checked;
    }
  }
  grid.visible = gridToggle.checked;
  axes.visible = axesToggle.checked;
}

function fitCameraToRobot() {
  const box = new THREE.Box3().setFromObject(robotRoot);
  if (box.isEmpty()) {
    orbit.target.set(0, 0, 0);
    orbit.radius = 3;
    updateCamera();
    return;
  }

  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const maxDimension = Math.max(size.x, size.y, size.z, 0.2);
  orbit.target.copy(center);
  orbit.radius = maxDimension * 2.4;
  camera.near = Math.max(maxDimension / 1000, 0.001);
  camera.far = Math.max(maxDimension * 100, 100);
  camera.updateProjectionMatrix();
  updateCamera();
}

function updateCamera() {
  orbit.phi = Math.max(0.08, Math.min(Math.PI - 0.08, orbit.phi));
  orbit.radius = Math.max(0.05, orbit.radius);
  const sinPhi = Math.sin(orbit.phi);
  camera.position.set(
    orbit.target.x + orbit.radius * sinPhi * Math.cos(orbit.theta),
    orbit.target.y + orbit.radius * Math.cos(orbit.phi),
    orbit.target.z + orbit.radius * sinPhi * Math.sin(orbit.theta)
  );
  camera.lookAt(orbit.target);
}

function resizeRenderer() {
  const rect = canvas.parentElement.getBoundingClientRect();
  renderer.setSize(rect.width, rect.height, false);
  camera.aspect = Math.max(rect.width / Math.max(rect.height, 1), 0.01);
  camera.updateProjectionMatrix();
}

function animate() {
  resizeRenderer();
  updateCamera();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

function panCamera(dx, dy) {
  const element = renderer.domElement;
  const distance = orbit.radius;
  const panSpeed = distance / Math.max(element.clientHeight, 1);
  const cameraDirection = new THREE.Vector3();
  camera.getWorldDirection(cameraDirection);
  const right = new THREE.Vector3().crossVectors(cameraDirection, camera.up).normalize();
  const up = new THREE.Vector3().crossVectors(right, cameraDirection).normalize();
  orbit.target.addScaledVector(right, -dx * panSpeed);
  orbit.target.addScaledVector(up, dy * panSpeed);
}

canvas.addEventListener('pointerdown', (event) => {
  orbit.dragging = true;
  orbit.button = event.button;
  orbit.lastX = event.clientX;
  orbit.lastY = event.clientY;
  canvas.setPointerCapture(event.pointerId);
});

canvas.addEventListener('pointermove', (event) => {
  if (!orbit.dragging) return;
  const dx = event.clientX - orbit.lastX;
  const dy = event.clientY - orbit.lastY;
  orbit.lastX = event.clientX;
  orbit.lastY = event.clientY;

  if (orbit.button === 2 || event.buttons === 2 || event.buttons === 4) {
    panCamera(dx, dy);
  } else {
    orbit.theta -= dx * 0.008;
    orbit.phi -= dy * 0.008;
  }
});

canvas.addEventListener('pointerup', (event) => {
  orbit.dragging = false;
  canvas.releasePointerCapture(event.pointerId);
});

canvas.addEventListener('wheel', (event) => {
  event.preventDefault();
  orbit.radius *= event.deltaY > 0 ? 1.12 : 0.88;
}, { passive: false });

canvas.addEventListener('contextmenu', (event) => event.preventDefault());

openFolderButton.addEventListener('click', () => folderInput.click());

folderInput.addEventListener('change', async () => {
  const files = Array.from(folderInput.files || []);
  clearRobot();

  if (files.length === 0) {
    folderName.textContent = 'Sin carpeta';
    setStatus('Listo para cargar una carpeta.');
    populateUrdfSelect();
    return;
  }

  indexFolderFiles(files);
  folderName.textContent = currentPackageRoot || 'Carpeta seleccionada';
  populateUrdfSelect();

  if (urdfOptions.length === 0) {
    setStatus('No encontre archivos .urdf o .xacro en la carpeta.', [
      'Selecciona la carpeta completa del paquete, no solo urdf/ o meshes/.'
    ]);
    return;
  }

  await loadSelectedUrdf();
});

urdfSelect.addEventListener('change', loadSelectedUrdf);
fitButton.addEventListener('click', fitCameraToRobot);
clearButton.addEventListener('click', () => {
  clearRobot();
  setStatus('Vista limpia.');
});

for (const toggle of [meshToggle, wireToggle, gridToggle, axesToggle]) {
  toggle.addEventListener('change', applyVisibilityState);
}

window.addEventListener('resize', resizeRenderer);

if (window.lucide) {
  window.lucide.createIcons();
}

updateCamera();
animate();
