.DEFAULT_GOAL := all
BUILD_NUMBER=$(shell date +%Y-%m-%d_%H%M%S)
BRANCH := $(shell git rev-parse --abbrev-ref HEAD)
PWD=$(shell pwd)
CP=$(shell command -v /usr/local/Cellar/coreutils/*/bin/gcp || command -v /bin/cp )
export

deps:
	go install honnef.co/go/tools/cmd/staticcheck@2025.1.1

prebuild: deps protobuf configs generates checks
	@echo "> other deps"
	go test ./... -v -coverprofile=target/coverage-report.out

protobuf:
	go run ./shared/gremlin_go/gremlinc/main.go -src ./protobufs -out ./target/generated-sources/protobuf -module norma_core/target/generated-sources/protobuf
	@echo "Generating Python Protobuf files..."
	@for dir in $(wildcard protobufs/*); do \
		if [ -d "$$dir" ]; then \
			folder_name=$$(basename $$dir); \
			python3 shared/gremlin_py/gremlin.py \
				--proto-root $$dir \
				--target-root target/gen_python/protobuf/$$folder_name \
				--project-root . \
				--gremlin-import-path "shared.gremlin_py.gremlin"; \
		fi \
	done

generates:
	go generate ./...

tests:
	go test -failfast ./...

checks:
	go vet ./...
	staticcheck ./...

clean:
	rm -rf target
	mkdir target

all: build-all

# —— Simulation targets ———————————————————————————————————————————————

# Ubuntu 24.04 enforces PEP 668 on system Python 3.12, so `pip install`
# without --break-system-packages fails. The sim-server is a pure-Python
# source layout and the existing system site-packages already has
# mujoco/numpy/pyyaml/pytest, so we run via PYTHONPATH instead of a
# true install.
SIM_PYTHONPATH := software/sim-server

.PHONY: sim-run
sim-run:
	./target/debug/station -c software/station/bin/station/station-sim.yaml

.PHONY: sim-standalone
sim-standalone:
	PYTHONPATH=$(SIM_PYTHONPATH) python3 -m norma_sim \
	  --manifest hardware/elrobot/simulation/manifests/norma/elrobot_follower.scene.yaml \
	  --socket /tmp/norma-sim-dev.sock \
	  --physics-hz 500 \
	  --publish-hz 100

.PHONY: sim-external
sim-external:
	./target/debug/station -c software/station/bin/station/station-sim-external.yaml

.PHONY: sim-shadow
sim-shadow:
	./target/debug/station -c software/station/bin/station/station-shadow.yaml

.PHONY: sim-test
sim-test: check-arch-invariants
	cargo test -p st3215-wire
	cargo test -p sim-runtime
	cargo test -p st3215-compat-bridge
	PYTHONPATH=$(SIM_PYTHONPATH) python3 -m pytest \
	    software/sim-server/tests/ \
	    hardware/elrobot/simulation/mujoco/elrobot_follower/tests/

# Architecture invariants enforced by grep. These MUST pass before any
# sim-related PR is merged — they encode the v2 architectural boundaries
# that distinguish MVP-1 from v1 (see the spec review for context).
.PHONY: check-arch-invariants
check-arch-invariants:
	@echo "Checking architecture invariants..."
	@if grep -r -i "st3215" software/sim-runtime/src/ > /dev/null; then \
	  echo "FAIL: sim-runtime has ST3215 reference"; exit 1; fi
	@if grep -r -i "st3215" software/sim-server/norma_sim/ > /dev/null; then \
	  echo "FAIL: norma_sim has ST3215 reference"; exit 1; fi
	@if grep -r -E "tokio|normfs|station_iface|StationEngine" software/drivers/st3215-wire/src/ > /dev/null; then \
	  echo "FAIL: st3215-wire has forbidden I/O dependency"; exit 1; fi
	@if grep -q "^pub trait WorldBackend" software/sim-runtime/src/backend/mod.rs; then \
	  echo "FAIL: WorldBackend trait must be pub(crate), not pub"; exit 1; fi
	@if cargo tree -p sim-runtime 2>/dev/null | grep -q "st3215-wire"; then \
	  echo "FAIL: sim-runtime transitively depends on st3215-wire"; exit 1; fi
	@echo "All architecture invariants hold ✓"

# —— MuJoCo web viewer (mjviser) ——————————————————————————————————————
# Browser-based MuJoCo viewer for WSL2 (mujoco.viewer needs OpenGL).
# Shows collision mesh, contact forces, joint sliders — open alongside
# station web for full debug experience.
#   station web: http://localhost:8889 (control)
#   mjviser:     http://localhost:8012 (3D debug)

MJVISER_PORT := 8012

.PHONY: viewer-elrobot
viewer-elrobot:
	mjviser hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml \
	  --port $(MJVISER_PORT)

.PHONY: viewer-so100
viewer-so100:
	mjviser hardware/elrobot/simulation/vendor/menagerie/trs_so_arm100/scene.xml \
	  --port $(MJVISER_PORT)

.PHONY: viewer-so101
viewer-so101:
	mjviser hardware/elrobot/simulation/vendor/therobotstudio/SO101/scene.xml \
	  --port $(MJVISER_PORT)

# —— Combined: station web + mjviser side by side ————————————————————
# Launches both in one command. Ctrl+C kills both.
#   http://localhost:8889  → station web (control)
#   http://localhost:8012  → mjviser (3D debug)

.PHONY: sim-debug-so101
sim-debug-so101:
	@echo "Starting station web (:8889) + mjviser (:8012)..."
	@echo "  http://localhost:8889  ← station (control + motor state)"
	@echo "  http://localhost:8012  ← mjviser (3D, synced with sim)"
	@PYTHONPATH=$(SIM_PYTHONPATH) ./target/debug/station \
	  -c software/station/bin/station/station-sim-therobotstudio.yaml \
	  --web 0.0.0.0:8889 \
	  --tcp 0.0.0.0:8888

.PHONY: sim-debug-elrobot
sim-debug-elrobot:
	@echo "Starting station web (:8889) + mjviser (:8012)..."
	@echo "  http://localhost:8889  ← station (control)"
	@echo "  http://localhost:8012  ← mjviser (3D debug)"
	@PYTHONPATH=$(SIM_PYTHONPATH) ./target/debug/station \
	  -c software/station/bin/station/station-sim.yaml \
	  --web 0.0.0.0:8889 & \
	mjviser hardware/elrobot/simulation/mujoco/elrobot_follower/scene.xml \
	  --port $(MJVISER_PORT); \
	kill %1 2>/dev/null; wait