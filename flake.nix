{
  description = "Rust development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" ];
        };
        rust-analyzer-mcp = pkgs.rustPlatform.buildRustPackage rec {
          pname = "rust-analyzer-mcp";
          version = "0.2.0";
          src = pkgs.fetchFromGitHub {
            owner = "zeenix";
            repo = "rust-analyzer-mcp";
            rev = "v${version}";
            hash = "sha256-brnzVDPBB3sfM+5wDw74WGqN5ahtuV4OvaGhnQfDqM0=";
          };
          cargoHash = "sha256-7t4bjyCcbxFAO/29re7cjoW1ACieeEaM4+QT5QAwc34=";
          nativeBuildInputs = [ pkgs.pkg-config ];
          buildInputs = [ pkgs.openssl ];
          doCheck = false;
        };
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            rustToolchain
            rust-analyzer-mcp
            pkg-config
            openssl
            libx11
            libxcursor
            libxrandr
            libXi
            libGL
            zenity
            samply
          ];

          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (with pkgs; [
            libx11
            libxcursor
            libxrandr
            libXi
            libGL
          ]);
          RUST_BACKTRACE = 1;

        };
      }
    );
}
