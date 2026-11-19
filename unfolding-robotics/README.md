# Unfolding Robotics：OpenArm 自主叠衣，公开模型与实机之间还差什么

核查：2026-10-22。Hugging Face / LeRobot 公开技术报告，未核实为同行评审论文。状态：报告、模型配置、数据元信息和部分源码审查；未下载权重、未运行策略、未完成实机复现或录像逐帧评分。

**[图片与原站视频播放器](media.html)** · [作者完整报告与演示](https://lerobot-robot-folding.hf.space/)。材料按现有十一项问题集中记录；搭建和实验计划只在 [Cadenza Robotics](https://github.com/Jeffaaay/cadenza-robotics) 维护。

## 1. 它是什么？人、机械臂和模型分别做什么

这是双臂 OpenArm 的实体 T 恤折叠工程。人通过 leader 示教，机器人作为 follower 执行；训练后的策略在自主阶段接替人发动作。作者使用的执行臂有上臂加长与定制夹爪。[技术报告](https://lerobot-robot-folding.hf.space/)

OpenArm Mini 是操作者使用的小型 Feetech leader，不是另一款承担叠衣负载的机械臂。其 [CAD、BOM 与说明](https://github.com/pkooij/open-arms-mini/tree/0628e081cfe105dc4156dba5099c2274c4aca2e8) 已公开，驱动在 LeRobot 中。

## 2. 它解决什么失败？

报告的早期策略难以把揉乱的衣服展开；更好的示范与进一步微调显著改变了结果。这个现象支持先检查数据与任务阶段，不能仅凭更换模型名称期待解决失败。[作者实验分析](https://lerobot-robot-folding.hf.space/)

我们的诊断例子：夹爪抓空后仍执行抬手，后面的折叠动作即使关节跟踪准确也无效。需要区分“夹爪接触没成功”“视觉没发现失败”和“策略没有恢复行为”；这是待测试的解释，不是作者已证明的全部失败原因。

## 3. 从示范到自主运行

公开 HQ 数据包含 1,200 条 episode，30 fps，三路 RGB 视频与状态/动作。状态和动作各 16 维，按右臂 J1–J7、右夹爪，再左臂 J1–J7、左夹爪排列。[数据元信息](https://huggingface.co/datasets/lerobot/high_quality_folding/blob/c9eb858d4b84e520edecbda84a3534c3c1e78436/meta/info.json)

模型配置为 `pi05`，处理器还接收任务文本。运行时把图像、状态和文本变成一段动作，经过配套处理器恢复为机器人目标。[模型配置及处理器](https://huggingface.co/lerobot-data-collection/folding_final/tree/695abe40dbf3aac04efda59c1501d748681fa0fb)

工程上要分别记录人的意图、发送的目标和实际反馈；它们不能因为共享同一个字段长度就被当成同一个量。

## 4. 实际改了哪里？

公开改件目录含加长件、夹爪接触件、相机支架和外盖；文件存在不等于兼容我们选定的 OpenArm 2.0 安装接口。[设计文件](https://huggingface.co/datasets/lerobot/openarms-hardware-modifications/tree/ffe34b93c070343042eb9412fbfeffce16139947)

代码也有具体转换：Mini 的 J6/J7 交换、左右方向翻转、夹爪范围换算均在驱动中处理。改变 leader 或 follower 时，需要重新验证这些映射。[Mini 实现](https://github.com/huggingface/lerobot/blob/1396b9fab7aecddd10006c33c47a487ffdcb54b4/src/lerobot/teleoperators/openarm_mini/openarm_mini.py)

## 5. 数学对应哪个真实动作？

教学表达：`关节目标 = 当前关节位置 + 策略给出的关节变化`，只是相对动作的直观解释，不是报告原式。

公开配置启用了相对动作，但排除了夹爪，并保存归一化、反归一化和恢复绝对动作的处理步骤。实际执行要按完整处理链处理；把网络中的数值直接发到电机，或把变化量加两遍，会改变动作含义。[预处理](https://huggingface.co/lerobot-data-collection/folding_final/blob/695abe40dbf3aac04efda59c1501d748681fa0fb/policy_preprocessor.json)、[后处理](https://huggingface.co/lerobot-data-collection/folding_final/blob/695abe40dbf3aac04efda59c1501d748681fa0fb/policy_postprocessor.json)

## 6. 证据是什么？

作者最佳模型：铺平 T 恤折叠 10/10；揉乱 T 恤展开、折叠并放置 8/10。每级五件衣服各试两次，合计 18/20。样本小，不能据此保证任意衣物或家庭环境中的可靠性。[实验与评估设置](https://lerobot-robot-folding.hf.space/)

作者试验不等于 Cadenza 实测；公开视频提供任务过程，不能替代全部失败分母、配置和统计不确定性。

## 7. 资源是否真的公开？

| 已检查的入口 | 实际检查到什么 | 尚未完成什么 |
|---|---|---|
| [folding_final](https://huggingface.co/lerobot-data-collection/folding_final/tree/695abe40dbf3aac04efda59c1501d748681fa0fb) | 权重文件列表、模型配置、处理器和训练配置 | 未下载约 7.47 GB 的权重；文件大小不是实际显存需求 |
| [HQ 数据](https://huggingface.co/datasets/lerobot/high_quality_folding/tree/c9eb858d4b84e520edecbda84a3534c3c1e78436) | 视频/Parquet 文件列表与元信息 | 未下载完整数据、未逐条审核质量 |
| [LeRobot v0.5.1](https://github.com/huggingface/lerobot/tree/1396b9fab7aecddd10006c33c47a487ffdcb54b4) | follower、双臂组合、Mini 配置与映射源码 | 未安装、未接硬件、未跑训练/推理 |
| [Mini](https://github.com/pkooij/open-arms-mini/tree/0628e081cfe105dc4156dba5099c2274c4aca2e8)及[改件](https://huggingface.co/datasets/lerobot/openarms-hardware-modifications/tree/ffe34b93c070343042eb9412fbfeffce16139947) | 设计文件入口、BOM 与装配说明 | 未做制造、结构或安装兼容验证 |

这比仅有演示视频更具复现条件，但不是一条已验证的“一键运行”命令。

## 8. 能直接搬到我们的 OpenArm 吗？

还不能这样承诺。本轮发现了可检查的差异：

- 模型卡通用示例写了 `act` 和 `so100_follower`；实际配置是 `pi05`。不能按示例设备名启动。
- v0.5.1 follower 的特征声明还含速度、力矩；策略需要的是 16 路位置。必须检查数据管线怎样选择字段。
- Mini README 的单端口示例，与锁定版本的双端口配置不同。
- 模型内 `rtc_config` 是空值；报告的运行时调度需要另外核对。训练配置的数据集 ID 与公开 HQ ID 不同，尚未证明两个快照完全对应。

上述是静态审查发现的复现事项，不把它们一概定性为上游 bug。完整工程检查表见 [Cadenza BASELINES](https://github.com/Jeffaaay/cadenza-robotics/blob/main/BASELINES.md)。

## 9. 后续与相关工作怎样看？

本轮未完成叠衣方法的系统综述。不能把“有人工纠错”“会自动重抓”或“筛选示范”单独当作新颖性证据。已有工作研究基于不确定性与人工负担选择何时求助，但其任务是辅助进食，不直接证明能解决布料操作。[相关论文](https://arxiv.org/abs/2602.10289)

## 10. 对我们研究有什么用？

它提供可检查的起点。我们先对齐已公开的参考硬件和接口，建立固定桌面的基准，再看抓空、滑落、展开失败是否足够频繁且可重复。若主要问题只是左右映射错误或超出可达范围，应先修复集成；若已排除这些问题，才评估失败恢复、少量数据适配或双臂控制。当前选型、已有电机与待核对的硬件版本在 [Cadenza 搭建清单](https://github.com/Jeffaaay/cadenza-robotics/blob/main/BUILD.md)维护，本文不另建一套计划。

## 11. 业界对照：Sunday Memo

Sunday 的 [ACT-2 技术报告](https://www.sunday.ai/blog/act-2-preview)称完成 778/785 次自主叠衣，覆盖九类衣物；Memo 可移动、调整高度并倾身。它与固定桌面 OpenArm 的硬件范围、数据与评估分布不同。该成绩是公司报告，不与本工程百分比直接排名，也不证明 Memo 使用同一套 OpenArm 机械结构。

## 图片与视频：带着问题看

![作者展示的双臂 OpenArm 工作台](https://lerobot-robot-folding.hf.space/_astro/ogp.k831iTAS.webp)

图片在线引用作者原图，用于识别系统，不是我们的实物照片。视频在线加载，不复制进仓库。

| 视频入口 | 要观察的问题 |
|---|---|
| [铺平 T 恤折叠](https://huggingface.co/datasets/huggingface/documentation-images/resolve/main/lerobot/level1.mp4) | 初态是否已摆好？手在哪里夹持？是否完整释放？ |
| [揉乱 T 恤整理](https://huggingface.co/datasets/huggingface/documentation-images/resolve/main/lerobot/level2.mp4) | 展开与折叠分别花多少步骤？重抓发生在哪里？ |
| [报告原页](https://lerobot-robot-folding.hf.space/) | 对照实验分母、视频标签和播放速度，不只看成功片段 |

本轮核对了原站视频地址和文件响应，未逐帧复核是否全程无干预。静态权重、数据和源码版本集中记录于 [Cadenza 版本文件](https://github.com/Jeffaaay/cadenza-robotics/blob/main/upstreams.lock.json)。
