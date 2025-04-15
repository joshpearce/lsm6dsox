{ nightly ? false }:
let
  nixpkgsBase = import (fetchTarball {
    name = "nixpkgs-stable-24.05";
    url = "https://github.com/NixOS/nixpkgs/archive/refs/tags/24.05.tar.gz";
    sha256 = "1lr1h35prqkd1mkmzriwlpvxcb34kmhc9dnr48gkm8hh089hifmx";
  });

  # Oxalica's Rust overlay gives us the rust-bin function which allows us
  # to select a specific Rust toolchain. Furthermore, we can configure
  # additional targets like shown below.
  rustOverlay = import (builtins.fetchTarball {
    name = "rust-overlay-2024-10-08";
    url = "https://github.com/oxalica/rust-overlay/archive/d216ade5a0091ce60076bf1f8bc816433a1fc5da.tar.gz";
    sha256 = "1cjmalanpbjf2zny100yjxslcpg6pfm5b6ih5gbv4ksiska9mr5g";
  });

  nixpkgs = nixpkgsBase {
    overlays = [ rustOverlay ];
  };

  # Choose between a specific Rust channel and 'latest', between stable and nightly
  # For nightly, we use a specific one so we do not download a new compiler every day
  rustChannel = (
    if nightly
    then nixpkgs.rust-bin.nightly."2024-10-08"
    # Stable Version of current nixpkgs
    else nixpkgs.rust-bin.stable."1.81.0"
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

