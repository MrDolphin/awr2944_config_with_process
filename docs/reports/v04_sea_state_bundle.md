# V0.4.27 海况仿真报告包

## 一键处理链

```text
多组 V0.4.23 point_cloud.h5
        ↓
V0.4.24 统计
        ↓
V0.4.25 图文报告
        ↓
V0.4.26 描述性判读
        ↓
一个可归档版本目录
```

## 使用方式

```powershell
python -m simulation.run_v04_sea_state_bundle `
  --input ss0 path\ss0\point_cloud.h5 `
  --input ss1 path\ss1\point_cloud.h5 `
  --input ss2 path\ss2\point_cloud.h5 `
  --input ss3 path\ss3\point_cloud.h5 `
  --output simulation\stages\v04_aoa_cfar_point_cloud\output\v04_27_sea_state_bundle
```

## 归档内容

目录包含 `stats/`、`visual_report/`、`interpretation/`、`bundle_summary.json` 和根目录 README，可作为一次实验批次的完整记录。

## 边界

报告包仍是描述性仿真分析，不自动给出真实海况等级、检测概率或实板 AoA 精度。
