/**
 * ============================================================================
 *  多軸無人機維修模擬小遊戲：無人機狀態與故障系統 v3 (Drone State & Fault System)
 * ============================================================================
 *
 *  v2 相較 v1 的變更
 *  ----------------
 *  - 新增零件：4 顆電調 ESC_FL/FR/BL/BR、接收機 Receiver（共 15 個零件）
 *  - 新增故障：馬達線序錯、槳螺帽鬆動、電調 MOS 燒毀 / 虛焊、電芯不平衡、
 *              飛控設定錯誤、接收機未對頻 / 天線斷裂
 *  - 故障分級 (tier 1~3)：關卡越高，越難診斷的故障才會出現
 *  - 連鎖故障 (cascade)：例如馬達燒毀時，同臂電調有機率一起燒掉
 *  - 起飛檢查回報 outcome：flip（翻車）/ crash（升空後墜落）/
 *    no_arm（無法解鎖）/ ok，供遊戲播放對應動畫
 *
 *  v3：新增機型 airframe
 *  - 'quad'      ：四軸訓練機（預設，15 個零件）
 *  - 'capricorn' ：陸軍「摩羯」旋翼型無人機（單主旋翼直升機構型，12 個零件）
 *                  新零件類型：旋翼 Rotor、舵機 Servo、光電酬載 Gimbal
 *                  新結果 mission：飛得起來，但酬載失效無法執行任務
 *  - 'altius'    ：Altius 600 管射式定翼無人機（摺疊主翼、X 型尾翼、後推槳，12 個零件）
 *                  新零件類型：主翼 Wing、摺疊槳 FoldProp
 *
 *  v4：新增六軸農藥機 'hexa'（H1~H6 六臂 + 噴灑系統 Sprayer，22 個零件）
 *      與颱風情境故障（tag: 'storm'，泡水腐蝕 / 泥沙卡入，需 opts.tags 開啟）
 *
 *  v5：新增燈光秀編隊機 'show'（四軸 + RTK 定位模組 GPS + LED 燈板，17 個零件）
 *
 *  架構總覽
 *  --------
 *  1. 常數層：STATUS、POSITIONS、PART_TYPES（資料驅動，故障清單都在這裡）
 *  2. 資料層：Drone.parts，每個零件 = { id, type, position, status }
 *  3. 邏輯層：generate_random_fault()、check_flight_ready()、repairPart()…
 *
 *  擴充方式
 *  --------
 *  A. 新增故障：STATUS 加代碼 → STATUS_LABEL 加中文 → PART_TYPES[類型].faults 加一筆
 *  B. 新增零件類型：PART_TYPES 加類型 → _buildParts() 建立零件
 *  C. 新增連鎖規則：在 CASCADES 加一筆
 *
 *  旋向慣例：四軸 X 型 Props In（俯視）
 *      FL 左前 = CW    FR 右前 = CCW
 *      BL 左後 = CCW   BR 右後 = CW
 * ============================================================================
 */

(function (global) {
  'use strict';

  // ==========================================================================
  // 1. 常數層
  // ==========================================================================

  /** 零件狀態代碼（程式邏輯一律比對代碼，顯示時再查 STATUS_LABEL） */
  const STATUS = Object.freeze({
    NORMAL: 'NORMAL',
    // --- 馬達 ---
    BURNT: 'BURNT',                     // 燒毀
    BEARING_SEIZED: 'BEARING_SEIZED',   // 軸承卡死
    WIRING_REVERSED: 'WIRING_REVERSED', // 三相線序接錯 → 馬達反轉
    // --- 螺旋槳 ---
    CRACKED: 'CRACKED',                 // 破裂
    REVERSED: 'REVERSED',               // 正反槳裝反
    LOOSE: 'LOOSE',                     // 槳螺帽鬆動
    // --- 電調 ---
    MOSFET_BURNT: 'MOSFET_BURNT',       // MOS 管燒毀
    COLD_SOLDER: 'COLD_SOLDER',         // 馬達線焊點虛焊
    // --- 電池 ---
    SWOLLEN: 'SWOLLEN',                 // 膨脹
    LOW_VOLTAGE: 'LOW_VOLTAGE',         // 電壓過低
    CELL_IMBALANCE: 'CELL_IMBALANCE',   // 電芯不平衡（單芯壞損）
    // --- 飛控板 ---
    SENSOR_FAILURE: 'SENSOR_FAILURE',   // 感測器損壞
    CONFIG_ERROR: 'CONFIG_ERROR',       // 設定錯誤（馬達順序 / 機型）
    // --- 接收機 ---
    NOT_BOUND: 'NOT_BOUND',             // 未對頻
    ANTENNA_BROKEN: 'ANTENNA_BROKEN',   // 天線斷裂
    // --- 旋翼（直升機）：CRACKED、LOOSE 共用 ---
    TRACKING: 'TRACKING',               // 槳葉循跡不一（兩片槳葉不在同一軌跡面）
    // --- 舵機（斜盤 / 尾舵）---
    GEAR_STRIPPED: 'GEAR_STRIPPED',     // 齒輪崩齒
    JITTER: 'JITTER',                   // 電位器磨損抖動
    SERVO_REVERSED: 'SERVO_REVERSED',   // 舵機方向設定相反
    // --- 光電酬載 ---
    LENS_DAMAGED: 'LENS_DAMAGED',       // 鏡頭窗刮傷 / 起霧
    THERMAL_FAIL: 'THERMAL_FAIL',       // 熱像模組失效
    // --- 定翼（Altius）---
    DEPLOY_JAM: 'DEPLOY_JAM',           // 主翼展開機構卡滯
    SKIN_DAMAGE: 'SKIN_DAMAGE',         // 主翼蒙皮破損
    HINGE_STUCK: 'HINGE_STUCK',         // 摺疊槳關節卡住
    // --- 六軸農藥機：噴灑系統 ---
    CLOGGED: 'CLOGGED',                 // 噴頭堵塞
    PUMP_DEAD: 'PUMP_DEAD',             // 水泵故障
    // --- 颱風災後（storm 標籤，只有颱風章節才會出現）---
    CORROSION: 'CORROSION',             // 泡水腐蝕
    MUD: 'MUD',                         // 泥沙卡入
    // --- 燈光秀編隊機：RTK 定位與 LED 燈板 ---
    RTK_FLOAT: 'RTK_FLOAT',             // RTK 只有浮動解（天線接頭鬆）
    COMPASS_ERR: 'COMPASS_ERR',         // 羅盤受干擾未校正
    LED_DEAD: 'LED_DEAD',               // 燈板不亮
    LED_FLICKER: 'LED_FLICKER',         // 燈板排線接觸不良閃爍
  });

  const STATUS_LABEL = Object.freeze({
    NORMAL: '正常',
    BURNT: '燒毀', BEARING_SEIZED: '軸承卡死', WIRING_REVERSED: '線序接錯（反轉）',
    CRACKED: '破裂', REVERSED: '正反槳裝反', LOOSE: '螺帽鬆動',
    MOSFET_BURNT: 'MOS 管燒毀', COLD_SOLDER: '焊點虛焊',
    SWOLLEN: '膨脹', LOW_VOLTAGE: '電壓過低', CELL_IMBALANCE: '電芯不平衡',
    SENSOR_FAILURE: '感測器損壞', CONFIG_ERROR: '設定錯誤',
    NOT_BOUND: '未對頻', ANTENNA_BROKEN: '天線斷裂',
    TRACKING: '循跡不一',
    GEAR_STRIPPED: '齒輪崩齒', JITTER: '抖動', SERVO_REVERSED: '方向設定相反',
    LENS_DAMAGED: '鏡頭窗刮傷', THERMAL_FAIL: '熱像模組失效',
    DEPLOY_JAM: '展開機構卡滯', SKIN_DAMAGE: '蒙皮破損', HINGE_STUCK: '摺疊關節卡住',
    CLOGGED: '噴頭堵塞', PUMP_DEAD: '水泵故障', CORROSION: '泡水腐蝕', MUD: '泥沙卡入',
    RTK_FLOAT: 'RTK 浮動解', COMPASS_ERR: '羅盤干擾', LED_DEAD: '燈板不亮', LED_FLICKER: '燈光閃爍',
  });

  const DIR = Object.freeze({ CW: 'CW', CCW: 'CCW' });

  /** 機臂位置（加位置即可擴充成六軸 / 八軸） */
  const POSITIONS = Object.freeze({
    FL: { label: '左前', expectedDir: DIR.CW },
    FR: { label: '右前', expectedDir: DIR.CCW },
    BL: { label: '左後', expectedDir: DIR.CCW },
    BR: { label: '右後', expectedDir: DIR.CW },
  });

  /** 直升機構型的位置（主旋翼俯視逆時針，尾旋翼反扭力） */
  const HELI_POSITIONS = Object.freeze({
    MAIN: { label: '主', expectedDir: DIR.CCW },
    TAIL: { label: '尾', expectedDir: DIR.CW },
    S1:   { label: '斜盤 1 號' },
    S2:   { label: '斜盤 2 號' },
    S3:   { label: '斜盤 3 號' },
  });
  /** Altius 600 的位置：後推槳（從機尾往前看順時針）與 X 型尾翼四片舵面 */
  const ALTIUS_POSITIONS = Object.freeze({
    PUSH: { label: '推進', expectedDir: DIR.CW },
    T1:   { label: '尾翼 1 號（右上）' },
    T2:   { label: '尾翼 2 號（右下）' },
    T3:   { label: '尾翼 3 號（左下）' },
    T4:   { label: '尾翼 4 號（左上）' },
  });
  /** 六軸農藥機（俯視順時針 H1 正前方起算，旋向 CW / CCW 交錯） */
  const HEXA_POSITIONS = Object.freeze({
    H1: { label: '1 號臂（前）',   expectedDir: DIR.CW },
    H2: { label: '2 號臂（右前）', expectedDir: DIR.CCW },
    H3: { label: '3 號臂（右後）', expectedDir: DIR.CW },
    H4: { label: '4 號臂（後）',   expectedDir: DIR.CCW },
    H5: { label: '5 號臂（左後）', expectedDir: DIR.CW },
    H6: { label: '6 號臂（左前）', expectedDir: DIR.CCW },
  });
  const ALL_POSITIONS = Object.freeze({ ...POSITIONS, ...HELI_POSITIONS, ...ALTIUS_POSITIONS, ...HEXA_POSITIONS });

  /** 機型 */
  const AIRFRAMES = Object.freeze({
    quad:      { label: '四軸訓練機' },
    capricorn: { label: '摩羯旋翼型無人機' },
    altius:    { label: 'Altius 600 管射式無人機' },
    hexa:      { label: '六軸農藥機' },
    show:      { label: '燈光秀編隊機' },
  });

  /**
   * 起飛結果類型，嚴重度由高到低：
   * - flip   ：旋向錯誤，一解鎖就翻車
   * - crash  ：動力系統異常，離地後失控墜落
   * - no_arm ：飛控不允許解鎖，螺旋槳完全不轉
   */
  const OUTCOME = Object.freeze({ OK: 'ok', MISSION: 'mission', FLIP: 'flip', CRASH: 'crash', NO_ARM: 'no_arm' });
  const OUTCOME_RANK = { ok: 0, mission: 1, no_arm: 2, crash: 3, flip: 4 };

  /**
   * 零件類型定義表（資料驅動核心）
   * faults 每筆：
   *   status  ：狀態代碼
   *   weight  ：抽中權重
   *   tier    ：難度等級 1~3（1 = 外觀看得出來，3 = 要用電腦或儀器才查得到）
   *   outcome ：此故障造成的起飛結果
   *   message ：起飛檢查訊息模板，{name} = 零件中文全名
   */
  const PART_TYPES = Object.freeze({
    // （四軸用 Motor/Prop/ESC/Battery/FlightController/Receiver；摩羯另加 Rotor/Servo/Gimbal；Altius 另加 Wing/FoldProp）
    Motor: {
      label: '馬達',
      faults: [
        { status: STATUS.BURNT,           weight: 2, tier: 1, outcome: OUTCOME.CRASH, message: '{name}燒毀，無法起飛' },
        { status: STATUS.BEARING_SEIZED,  weight: 2, tier: 2, outcome: OUTCOME.CRASH, message: '{name}軸承卡死，無法起飛' },
        { status: STATUS.WIRING_REVERSED, weight: 1, tier: 3, outcome: OUTCOME.FLIP,  message: '{name}線序接錯，轉向相反' },
        { status: STATUS.MUD,             weight: 2, tier: 1, outcome: OUTCOME.CRASH, message: '{name}卡入泥沙，轉速上不去', tag: 'storm' },
      ],
    },
    Prop: {
      label: '螺旋槳',
      faults: [
        { status: STATUS.CRACKED,  weight: 2, tier: 1, outcome: OUTCOME.CRASH, message: '{name}破裂，無法起飛' },
        { status: STATUS.REVERSED, weight: 2, tier: 1, outcome: OUTCOME.FLIP,  message: '{name}正反槳裝反（應為 {expected}，實裝 {actual}）' },
        { status: STATUS.LOOSE,    weight: 1, tier: 2, outcome: OUTCOME.CRASH, message: '{name}螺帽鬆動，起飛時槳飛脫' },
      ],
    },
    ESC: {
      label: '電調',
      faults: [
        { status: STATUS.MOSFET_BURNT, weight: 1, tier: 2, outcome: OUTCOME.CRASH, message: '{name}MOS 管燒毀，對應馬達不轉' },
        { status: STATUS.COLD_SOLDER,  weight: 1, tier: 3, outcome: OUTCOME.CRASH, message: '{name}焊點虛焊，馬達斷續失步' },
      ],
    },
    Battery: {
      label: '電池',
      faults: [
        { status: STATUS.SWOLLEN,        weight: 2, tier: 1, outcome: OUTCOME.NO_ARM, message: '{name}膨脹，有起火風險，禁止起飛' },
        { status: STATUS.LOW_VOLTAGE,    weight: 2, tier: 1, outcome: OUTCOME.NO_ARM, message: '{name}電壓過低，飛控拒絕解鎖' },
        { status: STATUS.CELL_IMBALANCE, weight: 1, tier: 2, outcome: OUTCOME.CRASH,  message: '{name}電芯不平衡，起飛後電壓驟降斷電' },
      ],
    },
    FlightController: {
      label: '飛控板',
      faults: [
        { status: STATUS.SENSOR_FAILURE, weight: 1, tier: 2, outcome: OUTCOME.NO_ARM, message: '{name}感測器損壞，無法解鎖' },
        { status: STATUS.CONFIG_ERROR,   weight: 1, tier: 3, outcome: OUTCOME.FLIP,   message: '{name}馬達順序設定錯誤，起飛時翻車' },
        { status: STATUS.CORROSION,      weight: 2, tier: 2, outcome: OUTCOME.NO_ARM, message: '{name}泡水腐蝕，開機自檢失敗', tag: 'storm' },
      ],
    },
    Receiver: {
      label: '接收機',
      faults: [
        { status: STATUS.NOT_BOUND,      weight: 1, tier: 2, outcome: OUTCOME.NO_ARM, message: '{name}未對頻，收不到遙控訊號' },
        { status: STATUS.ANTENNA_BROKEN, weight: 1, tier: 1, outcome: OUTCOME.CRASH,  message: '{name}天線斷裂，離地後訊號丟失失控' },
      ],
    },
    // ---------- 以下為直升機構型（摩羯）專用 ----------
    Rotor: {
      label: '旋翼',
      faults: [
        { status: STATUS.CRACKED,  weight: 2, tier: 1, outcome: OUTCOME.CRASH, message: '{name}槳葉破裂，無法起飛' },
        { status: STATUS.TRACKING, weight: 2, tier: 2, outcome: OUTCOME.CRASH, message: '{name}循跡不一，離地後劇烈振動失控' },
        { status: STATUS.LOOSE,    weight: 1, tier: 2, outcome: OUTCOME.CRASH, message: '{name}槳夾螺栓鬆動，起飛時槳葉甩脫' },
      ],
    },
    Servo: {
      label: '舵機',
      faults: [
        { status: STATUS.GEAR_STRIPPED,  weight: 2, tier: 1, outcome: OUTCOME.CRASH, message: '{name}齒輪崩齒，失去操縱' },
        { status: STATUS.JITTER,         weight: 1, tier: 2, outcome: OUTCOME.CRASH, message: '{name}抖動，懸停時發散失控' },
        { status: STATUS.SERVO_REVERSED, weight: 1, tier: 3, outcome: OUTCOME.FLIP,  message: '{name}方向設定相反，修正變成反向' },
      ],
    },
    Gimbal: {
      label: '光電酬載',
      faults: [
        { status: STATUS.LENS_DAMAGED, weight: 1, tier: 1, outcome: OUTCOME.MISSION, message: '{name}鏡頭窗刮傷，影像模糊無法偵蒐' },
        { status: STATUS.THERMAL_FAIL, weight: 1, tier: 2, outcome: OUTCOME.MISSION, message: '{name}熱像模組失效，夜間任務無法執行' },
      ],
    },
    // ---------- 以下為 Altius 600 專用 ----------
    Wing: {
      label: '主翼',
      faults: [
        { status: STATUS.SKIN_DAMAGE, weight: 2, tier: 1, outcome: OUTCOME.CRASH, message: '{name}蒙皮破損，升力不足墜落' },
        { status: STATUS.DEPLOY_JAM,  weight: 2, tier: 2, outcome: OUTCOME.CRASH, message: '{name}展開機構卡滯，出管後主翼沒有到位' },
      ],
    },
    FoldProp: {
      label: '摺疊槳',
      faults: [
        { status: STATUS.CRACKED,     weight: 2, tier: 1, outcome: OUTCOME.CRASH, message: '{name}槳葉破裂，推力不足' },
        { status: STATUS.HINGE_STUCK, weight: 1, tier: 2, outcome: OUTCOME.CRASH, message: '{name}摺疊關節卡住，槳葉張不開' },
      ],
    },
    // ---------- 以下為燈光秀編隊機專用 ----------
    GPS: {
      label: 'RTK 定位模組',
      faults: [
        { status: STATUS.RTK_FLOAT,   weight: 2, tier: 2, outcome: OUTCOME.MISSION, message: '{name}只有浮動解，編隊位置偏移' },
        { status: STATUS.COMPASS_ERR, weight: 1, tier: 3, outcome: OUTCOME.CRASH,   message: '{name}羅盤受干擾，起飛後繞圈飄走' },
      ],
    },
    LED: {
      label: 'LED 燈板',
      faults: [
        { status: STATUS.LED_DEAD,    weight: 2, tier: 1, outcome: OUTCOME.MISSION, message: '{name}不亮，表演時整架是暗的' },
        { status: STATUS.LED_FLICKER, weight: 1, tier: 2, outcome: OUTCOME.MISSION, message: '{name}排線接觸不良，燈光忽明忽暗' },
      ],
    },
    // ---------- 以下為六軸農藥機專用 ----------
    Sprayer: {
      label: '噴灑系統',
      faults: [
        { status: STATUS.CLOGGED,   weight: 2, tier: 1, outcome: OUTCOME.MISSION, message: '{name}噴頭堵塞，藥液噴不出來' },
        { status: STATUS.PUMP_DEAD, weight: 1, tier: 2, outcome: OUTCOME.MISSION, message: '{name}水泵故障，無法加壓噴灑' },
      ],
    },
  });

  /**
   * 連鎖故障規則：某零件出現 trigger 故障時，有 chance 機率讓 target 也故障。
   * target(part) 回傳受牽連零件的 ID。
   */
  const CASCADES = [
    // 馬達燒毀通常是過電流，同臂電調的 MOS 管也常一起燒掉
    { type: 'Motor', trigger: STATUS.BURNT, chance: 0.5, target: (p) => `ESC_${p.position}`, status: STATUS.MOSFET_BURNT },
    // 槳破裂多半是炸機撞擊，馬達軸也可能撞歪卡住
    { type: 'Prop', trigger: STATUS.CRACKED, chance: 0.3, target: (p) => `Motor_${p.position}`, status: STATUS.BEARING_SEIZED },
    // 主旋翼打到東西破裂時，衝擊常把斜盤舵機齒輪打崩
    { type: 'Rotor', trigger: STATUS.CRACKED, chance: 0.4, target: (p) => (p.position === 'MAIN' ? 'Servo_S1' : 'Servo_TAIL'), status: STATUS.GEAR_STRIPPED },
  ];

  /** 槳方向錯誤時的總結訊息（需求指定文字） */
  const MSG_PROP_DIRECTION = '螺旋槳方向錯誤，起飛時翻車';
  /** 直升機構型的方向錯誤總結訊息 */
  const MSG_HELI_CONTROL = '操縱方向錯誤，起飛時翻覆';
  /** Altius 的方向錯誤總結訊息 */
  const MSG_ALTIUS_CONTROL = '舵面方向錯誤，發射後螺旋下墜';

  // ==========================================================================
  // 2. 小工具
  // ==========================================================================

  function formatMessage(template, vars) {
    return template.replace(/\{(\w+)\}/g, (m, key) => (key in vars ? vars[key] : m));
  }
  function oppositeDir(dir) { return dir === DIR.CW ? DIR.CCW : DIR.CW; }
  function weightedPick(items, rng) {
    const total = items.reduce((s, it) => s + (it.weight ?? 1), 0);
    let r = rng() * total;
    for (const it of items) { r -= it.weight ?? 1; if (r < 0) return it; }
    return items[items.length - 1];
  }
  function sampleWithoutReplacement(arr, n, rng) {
    const copy = arr.slice(), count = Math.min(n, copy.length);
    for (let i = 0; i < count; i++) {
      const j = i + Math.floor(rng() * (copy.length - i));
      [copy[i], copy[j]] = [copy[j], copy[i]];
    }
    return copy.slice(0, count);
  }

  // ==========================================================================
  // 3. Drone 類別
  // ==========================================================================

  class Drone {
    /**
     * @param {Object}   [options]
     * @param {string}   [options.name='訓練機 01']
     * @param {Function} [options.rng=Math.random] 可傳入固定種子亂數，做可重現關卡
     * @param {boolean}  [options.verbose=true]    是否 console 輸出檢查結果
     * @param {string}   [options.airframe='quad'] 機型：'quad' | 'capricorn'
     */
    constructor(options = {}) {
      this.airframe = options.airframe ?? 'quad';
      if (!AIRFRAMES[this.airframe]) throw new Error(`未知機型：${this.airframe}`);
      this.name = options.name ?? '訓練機 01';
      this.rng = options.rng ?? Math.random;
      this.verbose = options.verbose ?? true;
      this.parts = this._buildParts();
      this.lastReport = null;
    }

    _buildParts() {
      const parts = {};
      const add = (type, position) => {
        const id = position ? `${type}_${position}` : type;
        parts[id] = { id, type, position, status: STATUS.NORMAL };
      };
      if (this.airframe === 'capricorn') {
        add('Motor', 'MAIN'); add('ESC', 'MAIN');
        add('Rotor', 'MAIN'); add('Rotor', 'TAIL');
        for (const pos of ['S1', 'S2', 'S3', 'TAIL']) add('Servo', pos);
        for (const type of ['Battery', 'FlightController', 'Receiver', 'Gimbal']) add(type, null);
        return parts;
      }
      if (this.airframe === 'show') {
        for (const pos of Object.keys(POSITIONS)) for (const type of ['Motor', 'Prop', 'ESC']) add(type, pos);
        for (const type of ['Battery', 'FlightController', 'Receiver', 'GPS', 'LED']) add(type, null);
        return parts;
      }
      if (this.airframe === 'hexa') {
        for (const pos of Object.keys(HEXA_POSITIONS)) for (const type of ['Motor', 'Prop', 'ESC']) add(type, pos);
        for (const type of ['Battery', 'FlightController', 'Receiver', 'Sprayer']) add(type, null);
        return parts;
      }
      if (this.airframe === 'altius') {
        add('Motor', 'PUSH'); add('ESC', 'PUSH'); add('FoldProp', 'PUSH'); add('Wing', null);
        for (const pos of ['T1', 'T2', 'T3', 'T4']) add('Servo', pos);
        for (const type of ['Battery', 'FlightController', 'Receiver', 'Gimbal']) add(type, null);
        return parts;
      }
      for (const pos of Object.keys(POSITIONS)) {
        for (const type of ['Motor', 'Prop', 'ESC']) {
          parts[`${type}_${pos}`] = { id: `${type}_${pos}`, type, position: pos, status: STATUS.NORMAL };
        }
      }
      for (const type of ['Battery', 'FlightController', 'Receiver']) {
        parts[type] = { id: type, type, position: null, status: STATUS.NORMAL };
      }
      return parts;
    }

    // ---------------- 查詢 ----------------

    /** Motor_FL → 「左前馬達」；Battery → 「電池」 */
    getPartName(partId) {
      const part = this.parts[partId], typeLabel = PART_TYPES[part.type].label;
      return part.position ? ALL_POSITIONS[part.position].label + typeLabel : typeLabel;
    }
    getStatusLabel(partId) { return STATUS_LABEL[this.parts[partId].status]; }

    /** 取得某零件目前故障的定義（含 tier / outcome），正常則回傳 null */
    getFaultDef(partId) {
      const p = this.parts[partId];
      return PART_TYPES[p.type].faults.find((f) => f.status === p.status) || null;
    }

    /** 螺旋槳實際安裝旋向 */
    getInstalledPropDir(partId) {
      const part = this.parts[partId];
      if (part.type !== 'Prop') throw new Error(`${partId} 不是螺旋槳`);
      const expected = ALL_POSITIONS[part.position].expectedDir;
      return part.status === STATUS.REVERSED ? oppositeDir(expected) : expected;
    }

    /** 馬達實際旋轉方向（線序接錯時相反） */
    getMotorSpinDir(partId) {
      const part = this.parts[partId];
      if (part.type !== 'Motor') throw new Error(`${partId} 不是馬達`);
      const expected = ALL_POSITIONS[part.position].expectedDir;
      return part.status === STATUS.WIRING_REVERSED ? oppositeDir(expected) : expected;
    }

    getFaultyParts() {
      return Object.values(this.parts).filter((p) => p.status !== STATUS.NORMAL).map((p) => p.id);
    }

    // ---------------- 故障產生 ----------------

    /**
     * 隨機讓 min~max 個「目前正常」的零件故障，並套用連鎖規則。
     *
     * @param {Object}  [opts]
     * @param {number}  [opts.min=1]
     * @param {number}  [opts.max=2]
     * @param {number}  [opts.maxTier=3]     只抽 tier ≤ maxTier 的故障（低關卡只出簡單故障）
     * @param {boolean} [opts.cascade=true]  是否套用連鎖故障
     * @param {string[]} [opts.tags=[]]      開啟情境故障，例如 ['storm']
     * @returns {Array<{id,name,status,label,cascade:boolean}>}
     *          本次產生的故障（cascade=true 表示是連鎖帶出的）
     */
    generate_random_fault(opts = {}) {
      const min = opts.min ?? 1, max = opts.max ?? 2, maxTier = opts.maxTier ?? 3;
      const useCascade = opts.cascade ?? true;
      const tags = opts.tags || [];
      // 帶 tag 的故障（例如颱風泡水）只在指定情境才會抽到
      const allowed = (p) => PART_TYPES[p.type].faults.filter((f) => f.tier <= maxTier && (!f.tag || tags.includes(f.tag)));

      // 1. 候選：目前正常、且有可用故障的零件
      const candidates = Object.values(this.parts)
        .filter((p) => p.status === STATUS.NORMAL && allowed(p).length > 0)
        .map((p) => p.id);
      if (!candidates.length) return [];

      // 2. 決定數量並抽零件
      const count = min + Math.floor(this.rng() * (max - min + 1));
      const chosen = sampleWithoutReplacement(candidates, count, this.rng);

      // 3. 依權重決定故障種類
      const result = [];
      const apply = (id, status, cascade) => {
        this.parts[id].status = status;
        result.push({ id, name: this.getPartName(id), status, label: STATUS_LABEL[status], cascade });
      };
      for (const id of chosen) apply(id, weightedPick(allowed(this.parts[id]), this.rng).status, false);

      // 4. 連鎖故障（只牽連目前正常的零件）
      if (useCascade) {
        for (const r of result.slice()) {
          const part = this.parts[r.id];
          for (const rule of CASCADES) {
            if (rule.type !== part.type || rule.trigger !== r.status) continue;
            const tid = rule.target(part);
            if (this.parts[tid] && this.parts[tid].status === STATUS.NORMAL && this.rng() < rule.chance) {
              apply(tid, rule.status, true);
            }
          }
        }
      }
      return result;
    }

    // ---------------- 起飛前檢查 ----------------

    /**
     * 檢查是否能成功起飛。
     * - 每個故障零件輸出一筆具體原因，例如「左前馬達燒毀，無法起飛」
     * - 只要有槳裝反、馬達線序錯或飛控馬達順序錯，額外輸出「螺旋槳方向錯誤，起飛時翻車」
     *
     * this.lastReport = {
     *   ready, reasons[], faults[], propDirectionError,
     *   outcome: 'ok' | 'flip' | 'crash' | 'no_arm'  // 取所有故障中最嚴重的
     * }
     * @returns {boolean}
     */
    check_flight_ready() {
      const reasons = [], faults = [];
      let propDirectionError = false, outcome = OUTCOME.OK;

      for (const part of Object.values(this.parts)) {
        if (part.status === STATUS.NORMAL) continue;
        const def = this.getFaultDef(part.id), name = this.getPartName(part.id);
        const vars = { name };
        if (part.type === 'Prop') {
          vars.expected = ALL_POSITIONS[part.position].expectedDir;
          vars.actual = this.getInstalledPropDir(part.id);
        }
        const message = def ? formatMessage(def.message, vars) : `${name}狀態異常，無法起飛`;
        const o = def ? def.outcome : OUTCOME.CRASH;
        if (o === OUTCOME.FLIP) propDirectionError = true;
        if (OUTCOME_RANK[o] > OUTCOME_RANK[outcome]) outcome = o;
        faults.push({ id: part.id, name, status: part.status, label: STATUS_LABEL[part.status], message, outcome: o });
        reasons.push(message);
      }
      if (propDirectionError) reasons.push({ capricorn: MSG_HELI_CONTROL, altius: MSG_ALTIUS_CONTROL }[this.airframe] || MSG_PROP_DIRECTION);

      const ready = reasons.length === 0;
      this.lastReport = { ready, reasons, faults, propDirectionError, outcome };

      if (this.verbose) {
        if (ready) console.log(`[${this.name}] 起飛前檢查通過，可以起飛`);
        else { console.log(`[${this.name}] 起飛前檢查未通過：`); reasons.forEach((r) => console.log(`  - ${r}`)); }
      }
      return ready;
    }

    // ---------------- 維修操作 ----------------

    /** 零件恢復正常；回傳原本是否故障 */
    repairPart(partId) {
      const part = this.parts[partId];
      if (!part) throw new Error(`找不到零件：${partId}`);
      const wasFaulty = part.status !== STATUS.NORMAL;
      part.status = STATUS.NORMAL;
      return wasFaulty;
    }

    /** 設定零件狀態（會擋掉不屬於該類型的狀態） */
    setPartStatus(partId, status) {
      const part = this.parts[partId];
      if (!part) throw new Error(`找不到零件：${partId}`);
      const allowed = [STATUS.NORMAL, ...PART_TYPES[part.type].faults.map((f) => f.status)];
      if (!allowed.includes(status)) throw new Error(`${this.getPartName(partId)} 不可能出現狀態：${status}`);
      part.status = status;
    }

    reset() {
      for (const part of Object.values(this.parts)) part.status = STATUS.NORMAL;
      this.lastReport = null;
    }

    toJSON() {
      const snapshot = {};
      for (const [id, part] of Object.entries(this.parts)) snapshot[id] = part.status;
      return { name: this.name, airframe: this.airframe, parts: snapshot };
    }
    loadJSON(data) {
      for (const [id, status] of Object.entries(data.parts)) if (this.parts[id]) this.setPartStatus(id, status);
    }
  }

  // ==========================================================================
  // 4. 匯出
  // ==========================================================================
  const api = { Drone, STATUS, STATUS_LABEL, DIR, POSITIONS, HELI_POSITIONS, ALTIUS_POSITIONS, HEXA_POSITIONS, ALL_POSITIONS, AIRFRAMES,
                PART_TYPES, CASCADES, OUTCOME, MSG_PROP_DIRECTION, MSG_HELI_CONTROL, MSG_ALTIUS_CONTROL };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else global.DroneFaultSystem = api;
})(typeof window !== 'undefined' ? window : globalThis);
