// Animación interactiva y cálculos (versión educativa)
// Carga al DOM
window.addEventListener("DOMContentLoaded", () => {
  document.getElementById("simulateBtn").addEventListener("click", onStart);
  document.getElementById("resetBtn").addEventListener("click", onReset);
  initCanvas();
});

/* ---------- Helpers para leer inputs ---------- */
function getVal(id){
  const el = document.getElementById(id);
  return Number(el.value);
}

/* ---------- Canvas & dibujo ---------- */
let canvas, ctx, W, H, scale=5;
let state = { robotPos:{x:0,y:0}, routePoints:[], boxes:[], animating:false, currentIndex:0 };
function initCanvas(){
  canvas = document.getElementById("warehouseCanvas");
  ctx = canvas.getContext("2d");
  W = canvas.width;
  H = canvas.height;
  drawScene();
}


function clearCanvas(){
  ctx.clearRect(0,0,W,H);
}

function worldToCanvas(p, bounds){
  // bounds {minX,maxX,minY,maxY} -> map to canvas with padding
  const pad = 40;
  const bx = bounds.minX, ex = bounds.maxX, by = bounds.minY, ey = bounds.maxY;
  const scaleX = (W - pad*2) / Math.max(1, ex - bx);
  const scaleY = (H - pad*2) / Math.max(1, ey - by);
  const s = Math.min(scaleX, scaleY);
  const cx = pad + (p.x - bx) * s;
  const cy = H - (pad + (p.y - by) * s); // invert y for canvas
  return {x:cx, y:cy};
}

function drawScene(highlightIndex=-1){
  clearCanvas();
  const bounds = computeBounds();
  // grid
  drawGrid(bounds);
  // base
  const baseC = worldToCanvas({x:0,y:0},bounds);
  ctx.fillStyle="#10b981"; ctx.beginPath(); ctx.arc(baseC.x, baseC.y, 10,0,Math.PI*2); ctx.fill();
  ctx.fillStyle="#042b4a"; ctx.font="12px  Inter";
  ctx.fillText("Base (0,0)", baseC.x+12, baseC.y+4);

  // boxes
  state.boxes.forEach((b,idx)=>{
    const c = worldToCanvas({x:b.x,y:b.y},bounds);
    const picked = b.picked;
    // box shadow / pickup animation
    ctx.save();
    ctx.translate(c.x,c.y);
    if(picked){
      ctx.globalAlpha = 0.5;
      ctx.fillStyle="#ffd166";
      ctx.fillRect(-10,-10,20,20);
      ctx.globalAlpha = 1;
      ctx.strokeStyle="#ff9f1c";
    } else {
      ctx.fillStyle="#ffb4a2";
      ctx.fillRect(-12,-12,24,24);
      ctx.strokeStyle="#ff6b6b";
    }
    ctx.strokeRect(-12,-12,24,24);
    ctx.restore();

    ctx.fillStyle="#042b4a";
    ctx.fillText(b.name, c.x-6, c.y-18);
  });

  // path
  if(state.routePoints.length>0){
    ctx.strokeStyle="rgba(37,99,235,0.9)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    state.routePoints.forEach((p,i)=>{
      const c = worldToCanvas(p,bounds);
      if(i===0) ctx.moveTo(c.x,c.y); else ctx.lineTo(c.x,c.y);
    });
    ctx.stroke();
  }

  // robot
  // === ROBOT ===
const robotC = worldToCanvas(state.robotPos, bounds);

// sombra
ctx.beginPath();
ctx.fillStyle = "rgba(2,6,23,0.08)";
ctx.ellipse(robotC.x, robotC.y + 12, 20, 8, 0, 0, Math.PI * 2);
ctx.fill();

// cuerpo principal
ctx.beginPath();
ctx.fillStyle = "#2563eb";
ctx.arc(robotC.x, robotC.y, 13, 0, Math.PI * 2);
ctx.fill();
ctx.strokeStyle = "#1e40af";
ctx.lineWidth = 2;
ctx.stroke();

// texto AGV
ctx.fillStyle = "#fff";
ctx.font = "12px Inter";
ctx.fillText("AGV", robotC.x - 13, robotC.y + 4);

// === CARGA SOBRE EL ROBOT ===
const pickedBoxes = state.boxes.filter(b => b.picked);
pickedBoxes.forEach((b, i) => {
  const h = 8; // altura entre cajas
  const yOffset = -(18 + i * (h + 6));
  ctx.fillStyle = "#fbbf24";
  ctx.strokeStyle = "#b45309";
  ctx.lineWidth = 1.4;
  ctx.fillRect(robotC.x - 10, robotC.y + yOffset - 8, 20, h + 8);
  ctx.strokeRect(robotC.x - 10, robotC.y + yOffset - 8, 20, h + 8);

  ctx.fillStyle = "#78350f";
  ctx.font = "10px Inter";
  ctx.fillText(b.name, robotC.x - 4, robotC.y + yOffset);
});

}

/* draw grid function */
function drawGrid(bounds){
  const step = Math.max(5, Math.round((bounds.maxX - bounds.minX)/10));
  ctx.lineWidth=0.6; ctx.strokeStyle="rgba(2,6,23,0.04)";
  for(let x = bounds.minX; x<=bounds.maxX; x+=step){
    const a = worldToCanvas({x:x,y:bounds.minY},bounds);
    const b = worldToCanvas({x:x,y:bounds.maxY},bounds);
    ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
  }
  for(let y = bounds.minY; y<=bounds.maxY; y+=step){
    const a = worldToCanvas({x:bounds.minX,y:y},bounds);
    const b = worldToCanvas({x:bounds.maxX,y:y},bounds);
    ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
  }
}

/* ---------- Cálculos físicos (idénticos a la versión Python) ---------- */
const g = 9.81;
function calcElevatorEnergy(params){
  const motor = params.elevator_power * params.elevator_time;
  const standby = params.standby_power * params.elevator_time;
  return (motor + standby) / 3600 / 1000; // kWh
}
function calcPhysics(m, params){
  const friction = params.friction_coeff * m * g;
  const tractionAccel = m * params.max_acceleration + friction;
  const tractionConst = friction;
  const powerAccel = tractionAccel * params.max_velocity;
  const powerConst = tractionConst * params.max_velocity;
  return { friction, tractionAccel, tractionConst, powerAccel, powerConst };
}
function calcSegment(dist, m, params){
  const accelDist = (params.max_velocity ** 2) / (2 * params.max_acceleration);
  const brakeDist = accelDist;
  let accelTime, cruiseTime, brakeTime, cruiseDist;
  if (dist <= accelDist + brakeDist){
    const reachedV = Math.sqrt(params.max_acceleration * dist);
    accelTime = reachedV / params.max_acceleration;
    brakeTime = accelTime;
    cruiseTime = 0; cruiseDist = 0;
  } else {
    accelTime = params.max_velocity / params.max_acceleration;
    brakeTime = accelTime;
    cruiseDist = dist - accelDist - brakeDist;
    cruiseTime = cruiseDist / params.max_velocity;
  }
  const totalTime = accelTime + cruiseTime + brakeTime;
  // energies in kWh
  const phys = calcPhysics(m, params);
  const e_accel = (phys.tractionAccel * accelDist) / params.propulsion_efficiency / 1000 / 3600;
  const e_cruise = (phys.tractionConst * (cruiseDist||0)) / params.propulsion_efficiency / 1000 / 3600;
  const e_brake = (phys.tractionConst * brakeDist) / params.propulsion_efficiency / 1000 / 3600;
  const e_standby = (params.standby_power / 1000) * (totalTime / 3600);
  return { totalTime, accelTime, cruiseTime, brakeTime, e_accel, e_cruise, e_brake, e_standby };
}

/* ---------- Optimización + preparación de segmentos (no animación) ---------- */
function prepareSimulation(){
  const params = {
    mass_robot:getVal("mass_robot"),
    mass_package:getVal("mass_package"),
    max_velocity:getVal("max_velocity"),
    max_acceleration:getVal("max_acceleration"),
    friction_coeff:getVal("friction_coeff"),
    standby_power:getVal("standby_power"),
    elevator_power:getVal("elevator_power"),
    elevator_efficiency:getVal("elevator_efficiency"),
    elevator_time:getVal("elevator_time"),
    elevator_height:getVal("elevator_height"),
    battery_energy:getVal("battery_energy"),
    propulsion_efficiency:getVal("propulsion_efficiency"),
  };

  const packages = [
    {name:"A", x:getVal("ax"), y:getVal("ay")},
    {name:"B", x:getVal("bx"), y:getVal("by")},
    {name:"C", x:getVal("cx"), y:getVal("cy")}
  ];

  function dist(p1,p2){ return Math.hypot(p2.x-p1.x, p2.y-p1.y); }
  const base = {name:"Base", x:0, y:0};
  const perms = permutations([0,1,2]);
  let best = null, bestE=Infinity, bestSegs=null;

  for(const perm of perms){
    let pos = base, mass = params.mass_robot;
    let segs=[];
    let totalE=0;
    for(let i=0;i<perm.length;i++){
      const pkg = packages[perm[i]];
      const d = dist(pos,pkg);
      const seg = calcSegment(d,mass,params);
      const elevE = calcElevatorEnergy(params);
      const segTotal = seg.e_accel + seg.e_cruise + seg.e_brake + seg.e_standby + elevE;
      segs.push({from:pos.name, to:pkg.name, x:pkg.x, y:pkg.y, dist:d, time:seg.totalTime, e_accel:seg.e_accel, e_cruise:seg.e_cruise, e_brake:seg.e_brake, e_standby:seg.e_standby, e_elev:elevE, e_total:segTotal});
      totalE += segTotal;
      pos = pkg; mass += params.mass_package;
    }
    // back to base
    const dback = dist(pos, base);
    const segBack = calcSegment(dback, mass, params);
    const backTotal = segBack.e_accel + segBack.e_cruise + segBack.e_brake + segBack.e_standby;
    segs.push({from:pos.name, to:"Base", x:base.x, y:base.y, dist:dback, time:segBack.totalTime, e_accel:segBack.e_accel, e_cruise:segBack.e_cruise, e_brake:segBack.e_brake, e_standby:segBack.e_standby, e_elev:0, e_total:backTotal});
    totalE += backTotal;
    if(totalE < bestE){ bestE=totalE; best=perm; bestSegs=segs; }
  }

  return {params, packages, segments:bestSegs, totalEnergy:bestE};
}

/* simple permutations */
function permutations(arr){
  if(arr.length<=1) return [arr];
  const out=[];
  for(let i=0;i<arr.length;i++){
    const rest = arr.slice(0,i).concat(arr.slice(i+1));
    for(const p of permutations(rest)) out.push([arr[i],...p]);
  }
  return out;
}

/* ---------- Animación paso a paso ---------- */
let animCtrl = { stop:false, speedScale: 0.45 }; // scale real seconds -> animation seconds
async function runAnimation(plan){
  // plan.segments: sequence with coordinates
  // prepare routePoints starting at Base(0,0)
  const route = [{x:0,y:0}, ...plan.segments.map(s => ({x:s.x,y:s.y})), {x:0,y:0}];
  state.routePoints = route;
  // initialize robot at base
  state.robotPos = {x:0,y:0};
  state.boxes = plan.packages.map(p => ({...p, picked:false}));
  state.animating = true;
  document.getElementById("animState").innerText = "Animando...";
  drawScene();

  // for bounds
  const bounds = computeBounds();

  for(let i=0;i<plan.segments.length;i++){
    const seg = plan.segments[i];
    // animate from current pos to seg.x,seg.y
    const from = {...state.robotPos};
    const to = {x:seg.x,y:seg.y};
    const simTime = Math.max(0.5, seg.time * animCtrl.speedScale); // seconds animation
    const frames = Math.round(simTime * 60);
    for(let f=1; f<=frames; f++){
      if(animCtrl.stop) { state.animating=false; document.getElementById("animState").innerText="Pausado"; return; }
      const t = f/frames;
      // simple linear interpolation for visualization (not physics-accurate)
      state.robotPos.x = from.x + (to.x - from.x) * t;
      state.robotPos.y = from.y + (to.y - from.y) * t;
      const prog = Math.round(((i + t) / plan.segments.length) * 100);
      document.getElementById("currentLeg").innerText = `${seg.from} → ${seg.to}`;
      document.getElementById("progress").innerText = `${prog}%`;
      drawScene();
      await waitFrame();
    }

    // arrive: mark box picked if not Base
    if(seg.to !== "Base"){
      const b = state.boxes.find(bb => bb.name === seg.to);
      if(b) {
        b.picked = true;
        // small arrival/pick animation: blink box scale
        await pickupAnimation(b);
      }
    }

    // small pause between legs
    await sleep(250);
  }

  // finished
  state.animating = false;
  document.getElementById("animState").innerText = "Finalizado";
  document.getElementById("currentLeg").innerText = "—";
  document.getElementById("progress").innerText = "100%";
  drawScene();
}

/* tiny helpers for animation timing */
function waitFrame(){ return new Promise(res => requestAnimationFrame(()=>res())); }
function sleep(ms){ return new Promise(res => setTimeout(res, ms)); }

/* pickup animation for a box (simple flash) */
async function pickupAnimation(box){
  const bounds = computeBounds();
  const c = worldToCanvas({x:box.x,y:box.y},bounds);
  for(let i=0;i<6;i++){
    // flash by drawing an overlay
    drawScene();
    ctx.save();
    ctx.translate(c.x,c.y);
    ctx.globalCompositeOperation = "lighter";
    ctx.fillStyle = "rgba(255, 199, 71, 0.35)";
    ctx.beginPath(); ctx.arc(0,0,28 + i*3,0,Math.PI*2); ctx.fill();
    ctx.restore();
    await sleep(60);
  }
  // Al final de pickupAnimation():
box.picked = true;
for (let i = 0; i < 8; i++) {
  drawScene();
  ctx.save();
  const c = worldToCanvas(state.robotPos, computeBounds());
  ctx.globalAlpha = 0.2;
  ctx.fillStyle = "rgba(250, 204, 21, 0.4)";
  ctx.beginPath(); ctx.arc(c.x, c.y - 20, 20 + i*2, 0, Math.PI*2); ctx.fill();
  ctx.restore();
  await sleep(40);
}
drawScene();

}

/* ---------- UI: Start / Reset ---------- */
async function onStart(){
  if(state.animating) return;
  animCtrl.stop = false;
  const plan = prepareSimulation();
  // attach packages to plan for animation
  plan.packages = plan.packages || ( (function(){ return [
    {name:"A", x:getVal("ax"), y:getVal("ay")},
    {name:"B", x:getVal("bx"), y:getVal("by")},
    {name:"C", x:getVal("cx"), y:getVal("cy")}
  ]})());
  // render results text & tables
  showResults(plan);
  // set robot initial pos and run animation
  state.robotPos = {x:0,y:0};
  await runAnimation(plan);
}

function onReset(){
  animCtrl.stop = true;
  state = { robotPos:{x:0,y:0}, routePoints:[], boxes:[], animating:false, currentIndex:0 };
  document.getElementById("animState").innerText = "Inactivo";
  document.getElementById("currentLeg").innerText = "—";
  document.getElementById("progress").innerText = "0%";
  document.getElementById("summary").innerHTML = "";
  document.getElementById("physics").innerHTML = "";
  document.querySelector("#segmentsTable tbody").innerHTML = "";
  document.querySelector("#segmentsTable tfoot").innerHTML = "";
  document.getElementById("energyBars").innerHTML = "";
  drawScene();
}

/* ---------- Mostrar resultados (texto + tablas + barras) ---------- */
function showResults(plan){
  const params = plan.params;
  // summary
  const totalWh = plan.totalEnergy * 1000;
  const batteryUsed = (plan.totalEnergy / params.battery_energy) * 100;
  document.getElementById("summary").innerHTML = `
    <p><b>Ruta óptima:</b> ${plan.segments.map(s=>s.from).join(" → ")} → Base</p>
    <p><b>Energía total:</b> ${totalWh.toFixed(2)} Wh (${plan.totalEnergy.toFixed(5)} kWh)</p>
    <p><b>Batería restante:</b> ${(100-batteryUsed).toFixed(2)}%</p>
  `;

  // physics (use robot + one package)
  const m_total = params.mass_robot + params.mass_package;
  const phys = calcPhysics(m_total, params);
  const elevE = calcElevatorEnergy(params);
  document.getElementById("physics").innerHTML = `
    <p>Fuerza de fricción: ${phys.friction.toFixed(2)} N</p>
    <p>Fuerza tracción (acelerar): ${phys.tractionAccel.toFixed(2)} N</p>
    <p>Fuerza tracción (constante): ${phys.tractionConst.toFixed(2)} N</p>
    <p>Potencia requerida (acelerar): ${phys.powerAccel.toFixed(2)} W</p>
    <p>Potencia requerida (crucero): ${phys.powerConst.toFixed(2)} W</p>
    <p>Energía elevador (por elevación): ${(elevE*1000).toFixed(3)} Wh</p>
  `;

  // segments table
  const tbody = document.querySelector("#segmentsTable tbody");
  tbody.innerHTML = "";
  const totals = {dist:0, time:0, wh:0, accel:0, cruise:0, brake:0, elev:0, standby:0};
  plan.segments.forEach(s=>{
    const row = document.createElement("tr");
    row.innerHTML = `<td>${s.from}→${s.to}</td><td>${s.dist.toFixed(2)}</td><td>${s.time.toFixed(2)}</td><td>${(s.e_total*1000).toFixed(2)}</td>`;
    tbody.appendChild(row);
    totals.dist+=s.dist; totals.time+=s.time; totals.wh+=s.e_total*1000;
    totals.accel += s.e_accel*1000; totals.cruise += s.e_cruise*1000; totals.brake += s.e_brake*1000;
    totals.elev += s.e_elev*1000; totals.standby += s.e_standby*1000;
  });
  const tfoot = document.querySelector("#segmentsTable tfoot");
  tfoot.innerHTML = `<tr><td><b>TOTAL</b></td><td><b>${totals.dist.toFixed(2)}</b></td><td><b>${totals.time.toFixed(2)}</b></td><td><b>${totals.wh.toFixed(2)}</b></td></tr>`;

  // energy bars (simple distribution)
  const energyBars = document.getElementById("energyBars");
  energyBars.innerHTML = "";
  const categories = [
    {label:"Aceleración", value:totals.accel},
    {label:"Crucero", value:totals.cruise},
    {label:"Frenado", value:totals.brake},
    {label:"Elevador", value:totals.elev},
    {label:"Standby", value:totals.standby}
  ];
  const totalVal = categories.reduce((s,c)=>s+c.value,0) || 1;
  const colors = ["#3b82f6","#10b981","#f59e0b","#ef4444","#8b5cf6"];
  categories.forEach((cat,i)=>{
    const row = document.createElement("div"); row.className="energyRow";
    row.innerHTML = `<div class="energyLabel">${cat.label}</div><div class="energyBar"><div class="energyFill" style="width:${(cat.value/totalVal*100).toFixed(1)}%;background:${colors[i]};"></div></div><div style="width:46px;text-align:right;font-size:13px">${cat.value.toFixed(2)} Wh</div>`;
    energyBars.appendChild(row);
  });
}

/* compute bounds for drawing (based on boxes & base) */
function computeBounds(){
  const xs = [0];
  const ys = [0];
  const ax = getVal("ax"), ay = getVal("ay");
  const bx = getVal("bx"), by = getVal("by");
  const cx = getVal("cx"), cy = getVal("cy");
  xs.push(ax,bx,cx); ys.push(ay,by,cy);
  const minX = Math.min(...xs) - 10, maxX = Math.max(...xs) + 10;
  const minY = Math.min(...ys) - 10, maxY = Math.max(...ys) + 10;
  return {minX, maxX, minY, maxY};
}

/* ---------- utility: draw initial scene ---------- */
(function drawInitial(){
  // set default boxes
  state.boxes = [
    {name:"A", x:getVal("ax"), y:getVal("ay"), picked:false},
    {name:"B", x:getVal("bx"), y:getVal("by"), picked:false},
    {name:"C", x:getVal("cx"), y:getVal("cy"), picked:false}
  ];
  drawScene();
})();

async function pickupAnimateBoxByName(name){
  const b = state.boxes.find(bb=>bb.name===name);
  if(b) await pickupAnimation(b);
}

/* ---------- small polyfills / end ---------- */
