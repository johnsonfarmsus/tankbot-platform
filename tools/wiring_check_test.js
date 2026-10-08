// Run the controller's wiring checker outside the page against known scenarios.
const fs = require('fs');
const html = fs.readFileSync(process.argv[2], 'utf8');
const js = html.split('<script>')[1].split('</script>')[0];
const grab = (start, end) => { const a = js.indexOf(start), b = js.indexOf(end, a); if (a < 0 || b < 0) throw new Error('missing ' + start); return js.slice(a, b); };
const code = grab('const SLOT_INFO =', 'const ROLE_OK =') +
  grab('// ================= wiring =================', 'let botView = "sensors";') +
  '\nmodule.exports = {wiring, sensorPins, wiringText, freeSlot, setDraft: d => { botDraft = d; }};';
const m = {exports: {}};
new Function('module', 'botDraft', code)(m, null);
let botDraft;
const W = m.exports;
// the checker reads a global botDraft: give it one per scenario
const run = (name, sensors, pins, expect) => {
  const src = code.replace('module.exports', 'var __x');
  const f = new Function('botDraft', src + '\nreturn {wiring, wiringText, freeSlot};');
  const api = f({sensors, pins});
  const w = api.wiring();
  const ok = !!expect(w, api);
  console.log((ok ? 'PASS ' : 'FAIL ') + name + (ok ? '' : '  errors=' + JSON.stringify(w.errors) + ' warns=' + JSON.stringify(w.warns)));
  return ok;
};
const lidar = {id: 'lidar', name: 'RPLidar', type: 'lidar', slot: 'LIDAR'};
const us = (id, slot, a, b) => ({id, name: 'Ultrasonic ' + id, type: 'ultrasonic', slot, pinA: a, pinB: b});
const bump = {id: 'bump1', name: 'Front bumper', type: 'bumper', slot: 'BUMP1'};
let all = true;
all &= run('the robot as built has no problems', [lidar, us('us1', 'US1'), bump], null,
  w => Object.keys(w.errors).length === 0 && Object.keys(w.warns).length === 0);
all &= run('two ultrasonics on the same default pins clash', [lidar, us('us1', 'US1'), us('us2', 'US1'), bump], null,
  w => w.errors.us1 && w.errors.us2 && /P14|P34/.test(w.errors.us2));
all &= run('a trigger on an input-only pin is refused', [lidar, us('us1', 'CUSTOM', 34, 35)], null,
  w => /input-only/.test(w.errors.us1 || ''));
all &= run('a motor pin moved onto a sensor pin clashes', [lidar, us('us1', 'US1')], {in1: 14},
  w => w.errors.motor && w.errors.us1);
all &= run('flash pins are refused', [lidar, us('us1', 'CUSTOM', 7, 34)], null,
  w => /flash/.test(w.errors.us1 || ''));
all &= run('the second ultrasonic default (P2) is allowed with a boot-pin caution', [lidar, us('us1', 'US1'), us('us2', 'US2'), bump], null,
  w => !w.errors.us2 && /P2/.test(w.warns.us2 || ''));
all &= run('two ToF sensors are refused (one serial port)', [lidar, {id: 't1', name: 'ToF 1', type: 'tof', slot: 'TOF'}, {id: 't2', name: 'ToF 2', type: 'tof', slot: 'CUSTOM', pinA: 23, pinB: 22}], null,
  w => /serial port/.test(w.errors.t2 || ''));
all &= run('a custom pin left unchosen blocks saving', [lidar, us('us1', 'CUSTOM', -1, 34)], null,
  w => /choose a pin/.test(w.errors.us1 || ''));
all &= run('a new ultrasonic gets the free default', [lidar, us('us1', 'US1'), {id: 'us2', name: 'New', type: 'ultrasonic'}], null,
  (w, api) => api.freeSlot({id: 'us2', type: 'ultrasonic'}) === 'US2');
all &= run('the pin budget counts the robot as built', [lidar, us('us1', 'US1'), bump], null,
  w => Object.keys(w.used).length === 11); // 6 motor + 2 lidar + 2 ultrasonic + 1 bumper
console.log(all ? 'ALL PASSED' : 'SOME FAILED');
