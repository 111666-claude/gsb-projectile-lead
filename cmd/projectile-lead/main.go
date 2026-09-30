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
	sample := flags.String("sample", "lead", "lead / noway / scale")
	if err := flags.Parse(args); err != nil {
		return 2
	}
	switch *sample {
	case "lead":
		engine := &projectile.Engine{}
		elapsed, dir, ok := engine.Lead(projectile.Vec{X: 100}, projectile.Vec{Y: 10}, 100)
		fmt.Fprintf(stdout, "hit=%v t=%.1f dirx=%.2f steps=%d\n", ok, elapsed, dir.X, engine.Steps())
	case "noway":
		// 目标用同样的速度朝背离方向跑，理论上永远追不上。
		engine := &projectile.Engine{}
		_, _, ok := engine.Lead(projectile.Vec{X: 100}, projectile.Vec{X: 100}, 100)
		fmt.Fprintf(stdout, "hit=%v steps=%d\n", ok, engine.Steps())
	case "scale":
		engine := &projectile.Engine{}
		for index := 0; index < 100000; index++ {
			engine.Lead(projectile.Vec{X: 100}, projectile.Vec{}, 100)
		}
		fmt.Fprintf(stdout, "steps=%d\n", engine.Steps())
	default:
		fmt.Fprintln(stderr, "需要 --sample lead|noway|scale")
		return 2
	}
	return 0
}

func main() {
	os.Exit(Run(os.Args[1:], os.Stdout, os.Stderr))
}
