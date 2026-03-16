package station

import (
	"fmt"
	"log/slog"
	"os"
	"strings"

	normfs "github.com/norma-core/normfs/normfs_go"

	commandspb "norma_core/target/generated-sources/protobuf/station/commands"
)

type StationClient interface {
	normfs.Client
}

// StreamEntry is re-exported from stream package
type StreamEntry = normfs.StreamEntry

func NewStationClient(server string) (StationClient, error) {
	streamsfsAddr := server
	if !strings.Contains(server, ":") {
		streamsfsAddr = fmt.Sprintf("%s:%d", server, 8888)
	}
	streamsfsLogger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	return normfs.NewClient(streamsfsAddr, streamsfsLogger)
}

func SendCommands(client normfs.Client, commands []*commandspb.DriverCommand) error {
	var bytes = make([][]byte, 0, len(commands))
	for _, cmd := range commands {
		bytes = append(bytes, cmd.Marshal())
	}
	_, err := client.EnqueuePack("commands", bytes)
	return err
}
