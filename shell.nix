{ nightly ? false }:
let
  nixpkgsBase = import (fetchTarball {
    name = "nixpkgs-stable-23.11";
    url = "https://github.com/NixOS/nixpkgs/archive/refs/tags/23.11.tar.gz";
    sha256 = "1ndiv385w1qyb3b18vw13991fzb9wg4cl21wglk89grsfsnra41k";
  });

  # Oxalica's Rust overlay gives us the rust-bin function which allows us
  # to select a specific Rust toolchain. Furthermore, we can configure
  # additional targets like shown below.
  rustOverlay = import (builtins.fetchTarball {
    name = "rust-overlay-2024-01-15";
    url = "https://github.com/oxalica/rust-overlay/archive/d681ac8a92a1cce066df1d3a5a7f7c909688f4be.tar.gz";
    sha256 = "1mrcbw14dlch89hnncs7pnm7lh4isqx7xywha6i97d8xs24spfvv";
  });

  nixpkgs = nixpkgsBase {
    overlays = [ rustOverlay ];
  };

  # Choose between a specific Rust channel and 'latest', between stable and nightly
  # For nightly, we use a specific one so we do not download a new compiler every day
  rustChannel = (
    if nightly
    then nixpkgs.rust-bin.nightly."2024-01-14"
    # Stable Version of current nixpkgs
    else nixpkgs.rust-bin.stable."1.73.0"
  ).default;

  # cargo-udeps only runs on nightly so we only need to include it if on nightly
  # this lets us skip downloading another nixpkgs when not required
  optionalCargoUdeps =
    if nightly
    then [ nixpkgs.cargo-udeps ]
    else [ ];
in
with nixpkgs;
mkShell {
  buildInputs = [
    git
    rustChannel
    cargo-deny
    gitlab-clippy
    cargo-tarpaulin
    cargo-readme
    cargo2junit
    nodePackages.cspell
  ] ++ optionalCargoUdeps;

  RUST_BACKTRACE = 1;
}

