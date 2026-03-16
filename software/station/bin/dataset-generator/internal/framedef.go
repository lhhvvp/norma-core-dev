package internal

type Frame struct {
	EpisodeStartNS               uint64  `parquet:"episode_start_ns"`
	GlobalFrameID                []byte  `parquet:"global_frame_id"`
	TimestampNsSinceEpisodeStart uint64  `parquet:"timestamp_ns_since_episode_start"`
	Joints                       []Joint `parquet:"joints"`
	Images                       []Image `parquet:"images"`
	Task                         string  `parquet:"task"`
}

type Joint struct {
	RangeMin     uint32  `parquet:"range_min"`
	RangeMax     uint32  `parquet:"range_max"`
	Position     uint32  `parquet:"position"`
	PositionNorm float32 `parquet:"position_norm"`
	Goal         uint32  `parquet:"goal"`
	GoalNorm     float32 `parquet:"goal_norm"`
	CurrentMA    uint32  `parquet:"current_ma"`
	Velocity     uint32  `parquet:"velocity"`
}

type Image struct {
	JPEG []byte `parquet:"jpeg"`
}
