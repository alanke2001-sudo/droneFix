// 簡易測試：node test.js
const assert = require('assert');
const { Drone, STATUS, PART_TYPES, MSG_PROP_DIRECTION, MSG_HELI_CONTROL } = require('./drone-fault-system.js');

// 1. 新機全正常（15 個零件）→ 可起飛
let d = new Drone({ verbose: false });
assert.strictEqual(Object.keys(d.parts).length, 15);
assert.strictEqual(d.check_flight_ready(), true);
assert.strictEqual(d.lastReport.outcome, 'ok');

// 2. 馬達燒毀訊息與 crash 結果
d.setPartStatus('Motor_FL', STATUS.BURNT);
assert.strictEqual(d.check_flight_ready(), false);
assert.ok(d.lastReport.reasons.includes('左前馬達燒毀，無法起飛'));
assert.strictEqual(d.lastReport.outcome, 'crash');

// 3. 槳裝反 → flip、旋向對調、翻車訊息只出現一次
d.reset();
d.setPartStatus('Prop_FL', STATUS.REVERSED);
d.setPartStatus('Motor_BR', STATUS.WIRING_REVERSED);
d.setPartStatus('Battery', STATUS.LOW_VOLTAGE);
assert.strictEqual(d.check_flight_ready(), false);
assert.strictEqual(d.getInstalledPropDir('Prop_FL'), 'CCW');
assert.strictEqual(d.getMotorSpinDir('Motor_BR'), 'CCW');
assert.strictEqual(d.lastReport.outcome, 'flip');
assert.strictEqual(d.lastReport.reasons.filter(r => r === MSG_PROP_DIRECTION).length, 1);

// 4. 只有 no_arm 類故障
d.reset(); d.setPartStatus('Receiver', STATUS.NOT_BOUND);
d.check_flight_ready(); assert.strictEqual(d.lastReport.outcome, 'no_arm');

// 5. 不合理狀態會被擋
assert.throws(() => d.setPartStatus('Battery', STATUS.BURNT));

// 6. maxTier=1 只會出 tier 1 故障，數量 1~2（不含連鎖）
for (let i = 0; i < 500; i++) {
  const x = new Drone({ verbose: false });
  const f = x.generate_random_fault({ maxTier: 1 });
  const primary = f.filter(z => !z.cascade);
  assert.ok(primary.length >= 1 && primary.length <= 2);
  primary.forEach(z => assert.strictEqual(x.getFaultDef(z.id).tier, 1));
}

// 7. 全部故障組合都抽得到，連鎖規則有觸發
const seen = new Set(); let cascades = 0;
for (let i = 0; i < 4000; i++) {
  const x = new Drone({ verbose: false });
  const f = x.generate_random_fault({ min: 2, max: 4, tags: ['storm'] });
  f.forEach(z => { seen.add(z.id + ':' + z.status); if (z.cascade) cascades++; });
  assert.strictEqual(x.getFaultyParts().length, f.length);
}
const expected = Object.values(new Drone({ verbose: false }).parts)
  .reduce((n, p) => n + PART_TYPES[p.type].faults.length, 0);
assert.strictEqual(seen.size, expected);
assert.ok(cascades > 0);

// 8. 存讀檔
const a = new Drone({ verbose: false }); a.generate_random_fault();
const b = new Drone({ verbose: false }); b.loadJSON(a.toJSON());
assert.deepStrictEqual(a.toJSON().parts, b.toJSON().parts);

// 9. 摩羯（直升機構型）：12 個零件、名稱、mission 結果、翻覆訊息
const h = new Drone({ verbose: false, airframe: 'capricorn' });
assert.strictEqual(Object.keys(h.parts).length, 12);
assert.strictEqual(h.check_flight_ready(), true);
assert.strictEqual(h.getPartName('Rotor_MAIN'), '主旋翼');
assert.strictEqual(h.getPartName('Servo_S2'), '斜盤 2 號舵機');
h.setPartStatus('Gimbal', STATUS.THERMAL_FAIL);
assert.strictEqual(h.check_flight_ready(), false);
assert.strictEqual(h.lastReport.outcome, 'mission');
h.setPartStatus('Servo_TAIL', STATUS.SERVO_REVERSED);
h.check_flight_ready();
assert.strictEqual(h.lastReport.outcome, 'flip');
assert.ok(h.lastReport.reasons.includes(MSG_HELI_CONTROL));
assert.throws(() => h.setPartStatus('Rotor_MAIN', STATUS.REVERSED));
assert.throws(() => new Drone({ airframe: 'x' }));
const hseen = new Set();
for (let i = 0; i < 4000; i++) {
  const x = new Drone({ verbose: false, airframe: 'capricorn' });
  x.generate_random_fault({ min: 2, max: 4, tags: ['storm'] }).forEach(z => hseen.add(z.id + ':' + z.status));
}
const hexpected = Object.values(h.parts).reduce((n, p) => n + PART_TYPES[p.type].faults.length, 0);
assert.strictEqual(hseen.size, hexpected);

// 10. Altius 600：12 個零件、翻覆訊息、全部故障抽得到
const { MSG_ALTIUS_CONTROL } = require('./drone-fault-system.js');
const al = new Drone({ verbose: false, airframe: 'altius' });
assert.strictEqual(Object.keys(al.parts).length, 12);
assert.strictEqual(al.getPartName('FoldProp_PUSH'), '推進摺疊槳');
assert.strictEqual(al.check_flight_ready(), true);
al.setPartStatus('Servo_T3', STATUS.SERVO_REVERSED); al.check_flight_ready();
assert.ok(al.lastReport.reasons.includes(MSG_ALTIUS_CONTROL));
const aseen = new Set();
for (let i = 0; i < 4000; i++) {
  const x = new Drone({ verbose: false, airframe: 'altius' });
  x.generate_random_fault({ min: 2, max: 4, tags: ['storm'] }).forEach(z => aseen.add(z.id + ':' + z.status));
}
const aexpected = Object.values(al.parts).reduce((n, p) => n + PART_TYPES[p.type].faults.length, 0);
assert.strictEqual(aseen.size, aexpected);

// 11. 六軸農藥機：22 個零件、旋向交錯、噴灑 mission、全部故障抽得到；storm 故障沒開 tag 不會出現
const hx = new Drone({ verbose: false, airframe: 'hexa' });
assert.strictEqual(Object.keys(hx.parts).length, 22);
assert.strictEqual(hx.getPartName('Prop_H3'), '3 號臂（右後）螺旋槳');
assert.strictEqual(hx.getInstalledPropDir('Prop_H2'), 'CCW');
hx.setPartStatus('Sprayer', STATUS.CLOGGED); hx.check_flight_ready();
assert.strictEqual(hx.lastReport.outcome, 'mission');
const xseen = new Set();
for (let i = 0; i < 6000; i++) {
  const x = new Drone({ verbose: false, airframe: 'hexa' });
  x.generate_random_fault({ min: 2, max: 4, tags: ['storm'] }).forEach(z => xseen.add(z.id + ':' + z.status));
  const y = new Drone({ verbose: false, airframe: 'hexa' });
  y.generate_random_fault({ min: 3, max: 4 }).forEach(z => assert.ok(z.status !== 'MUD' && z.status !== 'CORROSION'));
}
const xexpected = Object.values(hx.parts).reduce((n, p) => n + PART_TYPES[p.type].faults.length, 0);
assert.strictEqual(xseen.size, xexpected);

// 12. 燈光秀編隊機：17 個零件、LED mission、全部故障抽得到
const sh = new Drone({ verbose: false, airframe: 'show' });
assert.strictEqual(Object.keys(sh.parts).length, 17);
assert.strictEqual(sh.getPartName('GPS'), 'RTK 定位模組');
sh.setPartStatus('LED', STATUS.LED_DEAD); sh.check_flight_ready();
assert.strictEqual(sh.lastReport.outcome, 'mission');
const sseen = new Set();
for (let i = 0; i < 6000; i++) {
  const x = new Drone({ verbose: false, airframe: 'show' });
  x.generate_random_fault({ min: 2, max: 4, tags: ['storm'] }).forEach(z => sseen.add(z.id + ':' + z.status));
}
const sexpected = Object.values(sh.parts).reduce((n, p) => n + PART_TYPES[p.type].faults.length, 0);
assert.strictEqual(sseen.size, sexpected);

console.log(`全部測試通過（Altius ${aexpected} 種；四軸 ${expected} 種、摩羯 ${hexpected} 種零件故障組合，連鎖觸發 ${cascades} 次；六軸 ${xexpected} 種）`);
