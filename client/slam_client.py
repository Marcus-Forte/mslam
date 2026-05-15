import argparse
import logging

from viewer import SlamViewerClient

DEFAULT_SERVER_ADDR = "127.0.0.1:50052"
DEFAULT_VISER_PORT = 8080
DEFAULT_POINT_SIZE = 0.03


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Subscribe to the mslam gRPC server and render map snapshots and transformed scans in viser."
    )
    parser.add_argument(
        "--server-addr",
        default=DEFAULT_SERVER_ADDR,
        help="Address of the mslam gRPC server.",
    )
    parser.add_argument(
        "--viser-port",
        type=int,
        default=DEFAULT_VISER_PORT,
        help="Port for the local viser web UI.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=DEFAULT_POINT_SIZE,
        help="Rendered point size in viser.",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging verbosity.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    try:
        viewer = SlamViewerClient(
            server_addr=args.server_addr,
            viser_port=args.viser_port,
            point_size=args.point_size,
        )
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    viewer.run()


if __name__ == "__main__":
    main()
