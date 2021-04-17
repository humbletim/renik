wasm/renik-wasi.wasm: wasm/renik-wasi.cpp
	wasic++ -std=c++11 -I.. -I../polyfill -I../glm ../*.cpp ../renik/*.cpp wasm/*.cpp -o $@ -Wno-return-type -Wno-writable-strings -Wno-macro-redefined -fno-exceptions -Wno-logical-op-parentheses

wasm/renik-wasi.wasm.js: wasm/renik-wasi.wasm
	wasm2js $< > $@

test: renik-wasi.wasm
	wasmer $<
