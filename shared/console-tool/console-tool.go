// Package console_tool
// file was created on 22.08.2022 by ds
//
//	       ,.,
//	      MMMM_    ,..,
//	        "_ "__"MMMMM          ,...,,
//	 ,..., __." --"    ,.,     _-"MMMMMMM
//	MMMMMM"___ "_._   MMM"_."" _ """"""
//	 """""    "" , \_.   "_. ."
//	        ,., _"__ \__./ ."
//	       MMMMM_"  "_    ./
//	        ''''      (    )
//	 ._______________.-'____"---._.
//	  \                          /
//	   \________________________/
//	   (_)                    (_)
//
// ------------------------------------------------
package console_tool

import (
	"flag"
	"fmt"
	"norma_core/shared/ulimits"
	"os"
	"runtime"
	"strings"

	"github.com/rs/zerolog"
	"github.com/rs/zerolog/log"
)

func init() {
	zerolog.TimeFieldFormat = zerolog.TimeFormatUnix
	log.Logger = log.
		Output(zerolog.ConsoleWriter{Out: os.Stdout, TimeFormat: "02/01 15:04:05"}).
		Hook(LineInfoHook{})
	ulimits.SetupForHighLoad()
}

type LineInfoHook struct{}

func (h LineInfoHook) Run(e *zerolog.Event, l zerolog.Level, msg string) {
	if l >= zerolog.InfoLevel {
		_, file, line, ok := runtime.Caller(3)
		if ok {
			file = file[strings.Index(file, "norma_core/")+8:]
			e.Str("line", fmt.Sprintf("%s:%d", file, line))
		}
	}
}

func ConsoleInit(name string) zerolog.Logger {
	flag.Parse()

	if name != "" {
		return log.With().Str("app", name).Logger()
	} else {
		return log.Logger
	}
}
