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

# Regenerate MJCF from world.yaml manifest + URDF. Run after editing either.
.PHONY: regen-mjcf
regen-mjcf:
	python3 hardware/elrobot/simulation/worlds/gen.py