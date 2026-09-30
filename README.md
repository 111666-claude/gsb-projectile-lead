# projectile-lead

弹道预判：按目标速度解算相遇时刻，再给出射击方向，只用 Go 标准库。

```
go test ./...
go run ./cmd/projectile-lead --sample lead
go run ./cmd/projectile-lead --sample noway
go run ./cmd/projectile-lead --sample scale
```

## 口径（README 为准）

- **解相遇方程**：目标位置 `p`、速度 `v`，弹速 `s`，解 `|p + v*t| = s*t` 取最小正根，
  时间单位毫秒；没有正根时返回 `ok=false`（不许硬给一个命中）。
- **方向要按相遇点算**：射击方向是相遇点减去射击点的单位向量，不是当前夹角。
- **查询代价**：一次解算只做常数次运算，`Steps()` 是模拟/迭代步数，
  必须与距离、弹速都无关（不许逐帧模拟）。
- **规模**：单帧 10 万次解算、每帧 1 万发弹幕，单次 O(1)，内存 O(1)。

## 输出契约（不改格式）

```
hit=true t=1005.0 dirx=0.99 steps=1
hit=false steps=1
steps=100000
```

## 常量

```
StepMs   = 10
MaxSimMs = 10000
```

## 目录

```
lead.go                 弹道解算
cmd/projectile-lead     命令行入口
lead_test.go            go test 用例
```
