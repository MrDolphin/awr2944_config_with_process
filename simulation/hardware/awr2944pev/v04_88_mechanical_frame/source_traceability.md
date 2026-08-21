# V0.4.88 SPRR440 机械坐标证据追踪

- STEP：`C:\Users\56461\Downloads\2944p资料\sprr440a (1)\SPRR440\3D_PCB\PROC113D_BRD.step`
- STEP SHA-256：`4f94d0b39d451f2c6599faf4530dece865e8efc15bc088a384c2fa27ada0f271`
- 装配图：`C:\Users\56461\Downloads\2944p资料\sprr440a (1)\SPRR440\Assembly Drawings\PROC113D(001)_Assy.PDF`
- 装配图 SHA-256：`313499bd053476b3e6e9767f701f9030e354410d5c0ef326714d1955ba5a714b`
- 原理图：`C:\Users\56461\Downloads\2944p资料\sprr440a (1)\SPRR440\Schematics\PROC113D(001)_Sch.PDF`
- 原理图 SHA-256：`1391965069217a2fa1bbdd4d53d57a4bd7715b7edfb8e8db7c047432b29239cf`

## 已确认

STEP PCB BREP 的坐标包围盒为 `84.999840 × 124.999796 × 1.442720 mm`；顶面 z=0.000000 mm，底面 z=-1.442720 mm。STEP 使用毫米单位。

## 尚未确认

装配文件没有提供可直接读取的雷达前视基准、板面法向标注或安装旋转矩阵。因而当前只能冻结 PCB 局部坐标，不能把它直接当作雷达坐标；天线铜区几何中心也不能等同于电气相位中心。
