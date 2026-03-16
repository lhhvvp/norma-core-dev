//go:build js || windows

package ulimits

import (
	"fmt"
	"github.com/logrusorgru/aurora"
)

func SetupForHighLoad() {
	fmt.Printf("%s Skipping ulimit set because of platform mismatch\n",
		aurora.Yellow("DBG"))
}
