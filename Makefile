COMMIT_HASH := $(shell git rev-parse --short HEAD)
COMMIT_DATE := $(shell git log -1 --format=%cd --date=format:%Y%m%d)
DIRTY       := $(shell git diff --quiet || echo "-dirty")
RESULT_FILE := bench_results/bench_$(COMMIT_DATE)_$(COMMIT_HASH)$(DIRTY).txt

BENCH_COUNT ?= 5

.PHONY: test test-simd bench bench-simd bench-save bench-save-simd

## test: runs every test, the example included
test:
	go test ./...

## test-simd: runs every test built with the simd experiment
test-simd:
	GOEXPERIMENT=simd go test ./...

## bench: runs every benchmark once, with allocations, no test
bench:
	go test -run '^$$' -bench=. -benchmem -count=1 ./...

## bench-simd: runs every benchmark once, built with the simd experiment
bench-simd:
	GOEXPERIMENT=simd go test -run '^$$' -bench=. -benchmem -count=1 ./...

## bench-save: runs the benchmarks BENCH_COUNT times and keeps the raw output under bench_results/
bench-save:
	@mkdir -p bench_results
	go test -run '^$$' -bench=. -benchmem -count=$(BENCH_COUNT) ./... > $(RESULT_FILE)
	@echo "Results saved into: $(RESULT_FILE)"

## bench-save-simd: bench-save built with the simd experiment, into a file marked -simd
bench-save-simd:
	@mkdir -p bench_results
	GOEXPERIMENT=simd go test -run '^$$' -bench=. -benchmem -count=$(BENCH_COUNT) ./... > $(RESULT_FILE:.txt=-simd.txt)
	@echo "Results saved into: $(RESULT_FILE:.txt=-simd.txt)"
