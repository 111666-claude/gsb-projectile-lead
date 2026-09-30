// Command projectile-lead 跑弹道预判样例。
package main

import (
	"flag"
	"fmt"
	"io"
	"os"

	"example.com/projectile"
)

// Run 执行一次命令行调用，返回退出码。
func Run(args []string, stdout, stderr io.Writer) int {
	flags := flag.NewFlagSet("projectile-lead", flag.ContinueOnError)
	flags.SetOutput(stderr)
	sample := flags.String("sample", "radius", "radius / accel / noway / steps")
	if err := flags.Parse(args); err != nil {
		return 2
	}
	switch *sample {
	case "radius":
		// 正前方 100 米的固定靶，弹速 100 米每秒，命中半径 0.5 米。
		engine := &projectile.Engine{}
		elapsed, _, ok := engine.Lead(projectile.Target{P: projectile.Vec{X: 100}}, 100)
		fmt.Fprintf(stdout, "hit=%v t=%.1f steps=%d\n", ok, elapsed, engine.Steps())
	case "accel":
		// 目标以 200 米每秒平方的加速度冲向射手。
		engine := &projectile.Engine{}
		elapsed, _, ok := engine.Lead(projectile.Target{
			P: projectile.Vec{X: 100},
			A: projectile.Vec{X: -200},
		}, 100)
		fmt.Fprintf(stdout, "hit=%v t=%.1f\n", ok, elapsed)
	case "noway":
		// 同速逃逸并且还在向外加速。
		engine := &projectile.Engine{}
		elapsed, _, ok := engine.Lead(projectile.Target{
			P: projectile.Vec{X: 100},
			V: projectile.Vec{X: 100},
			A: projectile.Vec{X: 1000},
		}, 100)
		fmt.Fprintf(stdout, "hit=%v t=%.1f\n", ok, elapsed)
	case "steps":
		engine := &projectile.Engine{}
		for index := 0; index < 1000; index++ {
			engine.Lead(projectile.Target{P: projectile.Vec{X: 100}}, 100)
		}
		fmt.Fprintf(stdout, "steps=%d\n", engine.Steps())
	default:
		fmt.Fprintln(stderr, "需要 --sample radius|accel|noway|steps")
		return 2
	}
	return 0
}

func main() {
	os.Exit(Run(os.Args[1:], os.Stdout, os.Stderr))
}
