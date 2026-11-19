# Robotics Research Notes

每篇论文一个文件夹。笔记、公式、实验、后续方法、业界进展和原始来源链接都写在该文件夹的 README。公开版不保存第三方论文 PDF；请通过 arXiv、出版社或作者项目页访问原文。

## 当前项目路线

2026-10-22：工程工作由独立项目 [Cadenza Robotics](https://github.com/Jeffaaay/cadenza-robotics) 维护。已选择固定桌面的“双臂 OpenArm＋夹爪自主叠 T 恤”为第一阶段任务：逐臂调试 → 抓放与双臂示教 → 铺平衣物折叠 → 揉乱衣物整理 → 从失败中筛选控制或学习改进。直接参考 Unfolding Robotics 的公开工程；保留 OpenArm 原承力结构，先核对工作范围和接口差异。自定义外观限于可拆非承力外壳。

灵巧手选型、指尖改造和弹琴暂缓，保留为未来方向。LEAP v1、Tesollo 和开源五指改造均未被选为当前采购方案。实体库存、OpenArm 总预算和实机基准尚未确认；现有模型与演示不等于实体装配或论文结果已经复现。

搭建清单、平台路线和基准定义由 Cadenza 仓库维护；本仓库继续保存逐篇论文分析，避免重复维护多套计划。论文核查日期保留在各篇正文，项目路线更新不代表重新核实了全部文献结论。

## 论文资料

- [001 · Bunny-VisionPro](bunny-visionpro/README.md) — 核查至 2026-10-21，尚未复现。
- [002 · DITTO-X](ditto-x/README.md) — 主动外骨骼、接管与纠错学习；[图片与视频](ditto-x/media.html)。
- [003 · RealDexUMI](realdexumi/README.md) — 共享末端采集与跨机械臂部署；[图片与视频](realdexumi/media.html)。
- [004 · HandelBot](handelbot/README.md) — 实体弹琴与实机适配，当前保留为未来方向；[图片与视频](handelbot/media.html)，含 LEAP v1 动作对照。
- [005 · Unfolding Robotics](unfolding-robotics/README.md) — 双臂 OpenArm 自主叠衣技术报告；[图片与原站视频播放器](unfolding-robotics/media.html)。已审查公开模型、数据接口和驱动，尚未复现；不标作已确认同行评审论文。
- [006 · RC-LB-NMPC（IJRR 2016）](rc-lb-nmpc/README.md) — AMR path tracking、GP disturbance learning 与 uncertainty-aware constraints；包含四组公式、实验对照、代码验证和现有 AMR benchmark 的对应，尚未复现。
- [007 · UMI（RSS 2024）](umi/README.md) — 手持夹爪数据、相对动作、时延匹配与双臂叠衣；含直观例子、公式、实验分母和 OpenArm 适用边界。2026-10-25 核查正文、附录与公开实现，尚未复现。

以后新增论文直接按这个结构记录，不建立空模板、单独索引或占位目录。重要数字保留出处与测试条件，区分作者报告、公司报告和我们实测。

## Content and attribution

The written notes are original commentary and are not a substitute for the cited papers. Papers, project media, product names, and linked resources remain the property of their respective owners. Every factual claim should retain a primary-source link and a reproduction-status label.
