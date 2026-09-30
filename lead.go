// Package projectile 是弹道预判：目标带加速度运动，要在给定弹速与命中半径下找出最早的可行开火时刻。
// 缺陷：用固定 10ms 步长模拟（精度只有 10ms）、判据漏掉命中半径、忽略目标的加速度与速度、
// 无解时也返回命中、步数随飞行时间线性增长。
package projectile

import "math"

// SimStepMs 是模拟步长（毫秒）。
const SimStepMs = 10.0

// MaxFlightMs 是允许的最长飞行时间（毫秒）。
const MaxFlightMs = 5000.0

// HitRadius 是命中半径（米）。
const HitRadius = 0.5

// MaxIter 是一次解算允许的可行域求值次数上限。
const MaxIter = 64

// Vec 是二维坐标。
type Vec struct {
	X, Y float64
}

// Target 是目标状态：位置、速度与加速度。
type Target struct {
	P, V, A Vec
}

// Engine 是弹道解算器。
type Engine struct {
	steps int
}

// Steps 是累计的可行域求值次数（规模观测）。
func (e *Engine) Steps() int { return e.steps }

// Lead 返回最早的可行命中时刻（毫秒）与开火方向。
// 缺陷：只拿目标当前位置到射击点的距离和子弹行程比大小，既没有命中半径，也没有目标的
// 速度与加速度，模拟到上限还照样返回命中，精度受 10ms 步长限制。
func (e *Engine) Lead(target Target, bulletSpeed float64) (float64, Vec, bool) {
	distance := math.Hypot(target.P.X, target.P.Y)
	for elapsed := 0.0; elapsed <= MaxFlightMs; elapsed += SimStepMs {
		e.steps++
		if distance <= bulletSpeed*elapsed/1000.0 {
			return elapsed, direction(target.P), true
		}
	}
	return MaxFlightMs, direction(target.P), true
}

func direction(vector Vec) Vec {
	length := math.Hypot(vector.X, vector.Y)
	if length == 0 {
		return Vec{}
	}
	return Vec{X: vector.X / length, Y: vector.Y / length}
}
