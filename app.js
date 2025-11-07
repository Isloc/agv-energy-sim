window.addEventListener("DOMContentLoaded", () => {
  document.getElementById("simulateBtn").addEventListener("click", simulateAGV);
});

function getVal(id) {
  return parseFloat(document.getElementById(id).value);
}

function permutations(arr) {
  if (arr.length <= 1) return [arr];
  const result = [];
  arr.forEach((item, i) => {
    const rest = arr.slice(0, i).concat(arr.slice(i + 1));
    permutations(rest).forEach(p => result.push([item, ...p]));
  });
  return result;
}

function simulateAGV() {
  const g = 9.81;
  const params = {
    mass_robot: getVal("mass_robot"),
    mass_package: getVal("mass_package"),
    max_velocity: getVal("max_velocity"),
    max_acceleration: getVal("max_acceleration"),
    friction_coeff: getVal("friction_coeff"),
    standby_power: getVal("standby_power"),
    elevator_power: getVal("elevator_power"),
    elevator_efficiency: getVal("elevator_efficiency"),
    elevator_time: getVal("elevator_time"),
    elevator_height: getVal("elevator_height"),
    battery_energy: getVal("battery_energy"),
    propulsion_efficiency: getVal("propulsion_efficiency"),
  };

  const packages = [
    { name: "A", x: getVal("ax"), y: getVal("ay") },
    { name: "B", x: getVal("bx"), y: getVal("by") },
    { name: "C", x: getVal("cx"), y: getVal("cy") },
  ];

  function distance(p1, p2) {
    return Math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2);
  }

  function calcElevatorEnergy() {
    const motor = params.elevator_power * params.elevator_time;
    const standby = params.standby_power * params.elevator_time;
    return (motor + standby) / 3600 / 1000; // kWh
  }

  function calcPhysics(m) {
    const friction = params.friction_coeff * m * g;
    const tractionAccel = m * params.max_acceleration + friction;
    const tractionConst = friction;
    const powerAccel = tractionAccel * params.max_velocity;
    const powerConst = tractionConst * params.max_velocity;
    return { friction, tractionAccel, tractionConst, powerAccel, powerConst };
  }

  function calcSegment(dist, m) {
    const accelDist = (params.max_velocity ** 2) / (2 * params.max_acceleration);
    const brakeDist = accelDist;
    let accelTime, cruiseTime, brakeTime, cruiseDist;

    if (dist <= accelDist + brakeDist) {
      const reachedV = Math.sqrt(params.max_acceleration * dist);
      accelTime = reachedV / params.max_acceleration;
      brakeTime = accelTime;
      cruiseTime = 0;
      cruiseDist = 0;
    } else {
      accelTime = params.max_velocity / params.max_acceleration;
      brakeTime = accelTime;
      cruiseDist = dist - accelDist - brakeDist;
      cruiseTime = cruiseDist / params.max_velocity;
    }

    const totalTime = accelTime + cruiseTime + brakeTime;
    const phys = calcPhysics(m);

    // Energías en kWh → luego convertiremos a Wh
    const e_accel = (phys.tractionAccel * accelDist) / params.propulsion_efficiency / 1000 / 3600;
    const e_cruise = (phys.tractionConst * cruiseDist) / params.propulsion_efficiency / 1000 / 3600;
    const e_brake = (phys.tractionConst * brakeDist) / params.propulsion_efficiency / 1000 / 3600;
    const e_standby = (params.standby_power / 1000) * (totalTime / 3600);

    return {
      totalTime,
      accelTime,
      cruiseTime,
      brakeTime,
      e_accel,
      e_cruise,
      e_brake,
      e_standby,
    };
  }

  // --- Optimización de ruta ---
  const base = { name: "Base", x: 0, y: 0 };
  const allPerms = permutations([0, 1, 2]);
  let bestRoute = null;
  let minEnergy = Infinity;
  let bestSegments = [];

  for (const perm of allPerms) {
    let totalE = 0;
    let pos = base;
    let mass = params.mass_robot;
    let segs = [];

    for (let i = 0; i < perm.length; i++) {
      const pkg = packages[perm[i]];
      const d = distance(pos, pkg);
      const seg = calcSegment(d, mass);
      const elevE = calcElevatorEnergy(); // kWh
      const total_kWh = seg.e_accel + seg.e_cruise + seg.e_brake + seg.e_standby + elevE;
      totalE += total_kWh;
      segs.push({
        from: pos.name,
        to: pkg.name,
        dist: d,
        time: seg.totalTime,
        e_accel: seg.e_accel,
        e_cruise: seg.e_cruise,
        e_brake: seg.e_brake,
        e_elev: elevE,
        e_standby: seg.e_standby,
        e_total: total_kWh,
      });
      pos = pkg;
      mass += params.mass_package;
    }

    const back = distance(pos, base);
    const segBack = calcSegment(back, mass);
    const backTotal_kWh = segBack.e_accel + segBack.e_cruise + segBack.e_brake + segBack.e_standby;
    totalE += backTotal_kWh;
    segs.push({
      from: pos.name,
      to: "Base",
      dist: back,
      time: segBack.totalTime,
      e_accel: segBack.e_accel,
      e_cruise: segBack.e_cruise,
      e_brake: segBack.e_brake,
      e_elev: 0,
      e_standby: segBack.e_standby,
      e_total: backTotal_kWh,
    });

    if (totalE < minEnergy) {
      minEnergy = totalE;
      bestRoute = [base, ...perm.map(i => packages[i]), base];
      bestSegments = segs;
    }
  }

  // --- Resultados generales ---
  const batteryUsed = (minEnergy / params.battery_energy) * 100;
  const summaryDiv = document.getElementById("summary");
  summaryDiv.innerHTML = `
    <p><b>Ruta óptima:</b> ${bestRoute.map(p => p.name).join(" → ")}</p>
    <p><b>Energía total:</b> ${(minEnergy * 1000).toFixed(2)} Wh (${minEnergy.toFixed(5)} kWh)</p>
    <p><b>Batería restante:</b> ${(100 - batteryUsed).toFixed(2)}%</p>
  `;

  // --- Física calculada ---
  const m_total = params.mass_robot + params.mass_package;
  const phys = calcPhysics(m_total);
  const elevE = calcElevatorEnergy();
  document.getElementById("physics").innerHTML = `
    <p><b>Fuerza de Fricción:</b> ${phys.friction.toFixed(2)} N</p>
    <p><b>Fuerza Tracción (Acelerar):</b> ${phys.tractionAccel.toFixed(2)} N</p>
    <p><b>Fuerza Tracción (Constante):</b> ${phys.tractionConst.toFixed(2)} N</p>
    <p><b>Potencia Requerida (Acelerar):</b> ${phys.powerAccel.toFixed(2)} W</p>
    <p><b>Potencia Requerida (Crucero):</b> ${phys.powerConst.toFixed(2)} W</p>
    <p><b>Energía Motor Elevador:</b> ${(elevE * 1000).toFixed(3)} Wh (${elevE.toFixed(6)} kWh)</p>
  `;

  // --- Tabla detallada ---
  const tbody = document.querySelector("#segmentsTable tbody");
  const tfoot = document.querySelector("#segmentsTable tfoot");
  tbody.innerHTML = "";
  let totals = {
    dist: 0, time: 0,
    e_accel: 0, e_cruise: 0, e_brake: 0,
    e_elev: 0, e_standby: 0, e_total: 0
  };

  bestSegments.forEach(s => {
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>${s.from}→${s.to}</td>
      <td>${s.dist.toFixed(2)}</td>
      <td>${s.time.toFixed(2)}</td>
      <td>${(s.e_accel * 1000).toFixed(2)}</td>
      <td>${(s.e_cruise * 1000).toFixed(2)}</td>
      <td>${(s.e_brake * 1000).toFixed(2)}</td>
      <td>${(s.e_elev * 1000).toFixed(2)}</td>
      <td>${(s.e_standby * 1000).toFixed(2)}</td>
      <td>${(s.e_total * 1000).toFixed(2)}</td>
    `;
    tbody.appendChild(row);

    // sumas
    for (let k in totals) totals[k] += s[k] || 0;
  });

  tfoot.innerHTML = `
    <tr style="background:#dbeafe;font-weight:bold">
      <td>TOTAL</td>
      <td>${totals.dist.toFixed(2)}</td>
      <td>${totals.time.toFixed(2)}</td>
      <td>${(totals.e_accel * 1000).toFixed(2)}</td>
      <td>${(totals.e_cruise * 1000).toFixed(2)}</td>
      <td>${(totals.e_brake * 1000).toFixed(2)}</td>
      <td>${(totals.e_elev * 1000).toFixed(2)}</td>
      <td>${(totals.e_standby * 1000).toFixed(2)}</td>
      <td>${(totals.e_total * 1000).toFixed(2)}</td>
    </tr>
  `;
}
