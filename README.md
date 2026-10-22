# droneFix

留一點東西給後面的人

## 🎮 線上遊玩

**👉 https://alanke2001-sudo.github.io/droneFix/**

電腦、手機（iPhone / Android）瀏覽器打開就能玩，不用安裝。也可以下載 `index.html` 直接雙擊離線遊玩。

## 這是什麼

「無人機維修工作台」是一款多軸無人機故障排除模擬遊戲：

- 360° 3D 維修台，可旋轉檢視機體、量測、換件、試飛
- 機型：四軸、六軸農藥機、燈光秀編隊機、陸軍「摩羯」旋翼機、Altius 600
- 戰役 10 章、新手教學、每日任務、商店、技師等級與技能樹、故障圖鑑（102 條）
- 進度自動存在瀏覽器（localStorage）

## 檔案

| 檔案 | 說明 |
| --- | --- |
| `index.html` | 遊戲本體（單檔，內含字型、three.js、背景音樂，可離線） |
| `drone-fault-system.js` | 故障產生與判讀邏輯 |
| `test.js` | 故障系統單元測試（`node test.js`） |

## 授權與致謝

- 3D 引擎：[three.js](https://threejs.org/)（MIT）
- 背景音樂為原創曲，以 [Salamander Grand Piano](https://freepats.zenvoid.org/Piano/acoustic-grand-piano.html)（Alexander Holm，CC BY 3.0）取樣演奏
