// Package projectile 是弹道预判：按目标速度解算相遇时刻，再给出射击方向。
// 缺陷：逐帧模拟代替解算、只看目标当前位置（不预判）、无解时也返回命中、
// 迭代步数随距离增长、方向用当前夹角。
package projectile

import "math"

// StepMs 是模拟用的步长（毫秒）。
const StepMs = 10.0

// MaxSimMs 是模拟的最长时间（毫秒）。
const MaxSimMs = 10000.0

// Vec 是二维坐标与速度。
type Vec struct {
	X, Y float64
}

// Engine 是弹道解算器。
type Engine struct {
	steps int
}

// Steps 是累计模拟步数（规模观测）。
func (e *Engine) Steps() int { return e.steps }

// Lead 返回命中所需时间与射击方向。
// 缺陷：用 10ms 步长逐帧模拟（步数随距离线性增长），而且只判断目标当前位置是否已经进入
// 弹道射程，完全不考虑目标自己的速度；模拟到上限也照样返回命中。
func (e *Engine) Lead(target, targetVel Vec, bulletSpeed float64) (float64, Vec, bool) {
	distance := math.Hypot(target.X, target.Y)
	for elapsed := 0.0; elapsed <= MaxSimMs; elapsed += StepMs {
		e.steps++
		if distance <= bulletSpeed*elapsed/1000.0 {
			return elapsed, direction(target), true
		}
	}
	return MaxSimMs, direction(target), true
}

func direction(target Vec) Vec {
	length := math.Hypot(target.X, target.Y)
	if length == 0 {
		return Vec{}
	}
	return Vec{X: target.X / length, Y: target.Y / length}
}
