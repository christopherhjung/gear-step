#!/bin/sh
# Build the WebAssembly module and the web app.
#
#   ./build.sh          release wasm -> web/gear.wasm and dist/gear-step.html
#   ./build.sh serve    the same, then serve web/ on http://localhost:8080
#
# Needs only a stable Rust toolchain with the wasm32-unknown-unknown target:
#   rustup target add wasm32-unknown-unknown

set -e
cd "$(dirname "$0")"

echo "-> cargo build --release --target wasm32-unknown-unknown"
cargo build --release --target wasm32-unknown-unknown

WASM=target/wasm32-unknown-unknown/release/gear_step.wasm
cp "$WASM" web/gear.wasm
echo "-> web/gear.wasm  $(wc -c < web/gear.wasm | tr -d ' ') bytes"

# Optional: shrink it if wasm-opt (binaryen) happens to be installed. Written
# to the side first, so an old or unhappy wasm-opt cannot leave a broken module
# behind - we just keep the plain build.
if command -v wasm-opt >/dev/null 2>&1; then
    if wasm-opt -Oz --enable-bulk-memory -o web/gear.opt.wasm "$WASM" 2>/dev/null; then
        mv web/gear.opt.wasm web/gear.wasm
        echo "-> wasm-opt   $(wc -c < web/gear.wasm | tr -d ' ') bytes"
    else
        rm -f web/gear.opt.wasm
        echo "-> wasm-opt failed, keeping the plain build"
    fi
fi

mkdir -p dist
python3 - <<'PY'
import base64, pathlib, re
web = pathlib.Path("web")
html = (web / "index.html").read_text()
css = (web / "style.css").read_text()
js = (web / "app.js").read_text()
b64 = base64.b64encode((web / "gear.wasm").read_bytes()).decode()

# The icon too - a single file page cannot fetch favicon.svg from anywhere.
# base64 rather than percent escaping: the SVG is full of quotes and a #rrggbb
# per colour, and one missed escape silently ends the href attribute.
icon = base64.b64encode((web / "favicon.svg").read_bytes().strip()).decode()
html = html.replace('<link rel="icon" href="favicon.svg" type="image/svg+xml">',
                    '<link rel="icon" href="data:image/svg+xml;base64,' + icon + '">')

html = html.replace('<link rel="stylesheet" href="style.css">',
                    "<style>\n" + css + "\n</style>")
html = html.replace('<script type="module" src="app.js"></script>',
                    '<script>const GEAR_WASM_B64="' + b64 + '";</script>\n'
                    '<script type="module">\n' + js + '\n</script>')
out = pathlib.Path("dist/gear-step.html")
out.write_text(html)
print("-> dist/gear-step.html  %d bytes (self contained)" % out.stat().st_size)
PY

if [ "$1" = "serve" ]; then
    echo "-> http://localhost:8080"
    cd web && python3 -m http.server 8080
fi
