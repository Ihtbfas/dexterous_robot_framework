# Demo Media

这个目录用于保存 README 中的**轻量效果预览素材**。

完整高清视频不建议长期直接存入 Git 仓库；推荐上传到对应版本的 GitHub Release，并让 README 中的预览 GIF 点击后跳转到完整视频。

## 当前 Demo

当前第一条正式 Demo：

```text
WAM7 + Linker Hand L20
Isaac Sim
Tabletop Grasp & Lift
m1.6-motion-pacing-height-v1
```

推荐文件名：

```text
docs/media/demos/
└── m1_6_wam7_l20_grasp_lift_preview.gif
```

GitHub Release 中推荐：

```text
Release:
m1.6-motion-pacing-height-v1

Asset:
m1_6_wam7_l20_grasp_lift.mp4
```

## 预览素材建议

README 预览建议：

```text
时长：8～15 s
分辨率：宽度约 720～960 px
帧率：10～15 fps
内容：保留接近 / 抓握 / 举升 / 悬停的关键动作
目标：优先保证“第一次打开 README 就能看懂做到了什么”
```

尽量裁掉：

- Isaac Sim 启动画面；
- 长时间静止等待；
- 无关 UI；
- 实验前后的空白时间。

## 推荐展示方式

主 README 中使用：

```markdown
[![WAM7 + Linker L20 Grasp & Lift Demo](
docs/media/demos/m1_6_wam7_l20_grasp_lift_preview.gif
)](
https://github.com/<OWNER>/<REPO>/releases/download/
m1.6-motion-pacing-height-v1/
m1_6_wam7_l20_grasp_lift.mp4
)
```

实际使用时请把 URL 写成一行，并将：

```text
<OWNER>/<REPO>
```

替换为真实 GitHub 仓库。

## 后续 Demo Gallery 命名

建议保持：

```text
<stage>_<arm>_<hand>_<task>_preview.gif
<stage>_<arm>_<hand>_<task>.mp4
```

例如：

```text
m2_wam7_l20_grasp_lift_preview.gif
m2_wam7_l20_grasp_lift.mp4

m3_wam7_l20_real_grasp_lift_preview.gif
m3_wam7_l20_real_grasp_lift.mp4
```

这样 README 中可以长期保持：

```text
Isaac
MuJoCo
Real
```

三类效果的并列展示，而不会让源码仓库被大量高清视频占据。
