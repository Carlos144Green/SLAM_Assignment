#!/usr/bin/env python3
"""
=====================================================================
 * MIT License
 * 
 * Copyright (c) 2026 Omni Instrument Inc.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ===================================================================== 
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import os
import shutil
import sys
import tempfile
from collections.abc import Iterator, Sequence
from pathlib import Path


REPO_ID = "OmniInstrument/SLAM_project"
REPO_TYPE = "dataset"
REVISION = "main"

ROS1_FILES = (
    "ros1/omni_stereo_20260425_215907Z.bag",
    "ros1/omni_stereointertial_20260425_220304Z.bag",
)

ROS2_PREFIX = "ros2/omni_vio_20260425_220737Z_with_gt"


@dataclass(frozen=True)
class HuggingFaceRepository:
    repo_id: str
    repo_type: str
    revision: str

    @contextmanager
    def snapshot(self, patterns: Sequence[str]) -> Iterator[Path]:
        from huggingface_hub import snapshot_download

        tmp_dir = Path(tempfile.mkdtemp(prefix="slam-hf-snapshot-"))
        try:
            snapshot_dir = Path(
                snapshot_download(
                    repo_id=self.repo_id,
                    repo_type=self.repo_type,
                    revision=self.revision,
                    allow_patterns=list(patterns),
                    local_dir=tmp_dir,
                )
            )
            yield snapshot_dir
        finally:
            shutil.rmtree(tmp_dir, ignore_errors=True)


@dataclass(frozen=True)
class FileCopier:
    @staticmethod
    def downloaded(path: Path) -> bool:
        return path.is_file() and path.stat().st_size > 0

    def copy(self, source: Path, destination: Path) -> None:
        if self.downloaded(destination):
            print(f"Already downloaded: {destination}")
            return

        destination.parent.mkdir(parents=True, exist_ok=True)
        tmp_destination = destination.with_name(f".{destination.name}.tmp-{os.getpid()}")
        shutil.copy2(source, tmp_destination)
        os.replace(tmp_destination, destination)
        print(f"Downloaded: {destination}")


@dataclass(frozen=True)
class Ros1DatasetDownloader:
    repository: HuggingFaceRepository
    copier: FileCopier
    files: Sequence[str] = ROS1_FILES

    def download(self, data_dir: Path) -> None:
        missing_files = [
            repo_path
            for repo_path in self.files
            if not self.copier.downloaded(data_dir / Path(repo_path).name)
        ]
        if not missing_files:
            print(f"ROS 1 dataset already exists in {data_dir}")
            return

        with self.repository.snapshot(missing_files) as snapshot_dir:
            for repo_path in self.files:
                source = snapshot_dir / repo_path
                destination = data_dir / Path(repo_path).name
                if source.is_file():
                    self.copier.copy(source, destination)


@dataclass(frozen=True)
class Ros2DatasetDownloader:
    repository: HuggingFaceRepository
    copier: FileCopier
    prefix: str = ROS2_PREFIX

    def download(self, data_dir: Path) -> None:
        output_dir = data_dir / Path(self.prefix).name
        marker = output_dir / ".download_complete"
        if marker.is_file():
            print(f"ROS 2 dataset already exists in {output_dir}")
            return

        with self.repository.snapshot([f"{self.prefix}/**"]) as snapshot_dir:
            source_dir = snapshot_dir / self.prefix
            if not source_dir.is_dir():
                raise RuntimeError(f"Snapshot did not contain {self.prefix}")

            for source in source_dir.rglob("*"):
                if source.is_file():
                    self.copier.copy(source, output_dir / source.relative_to(source_dir))

            output_dir.mkdir(parents=True, exist_ok=True)
            marker.write_text("complete\n", encoding="utf-8")


@dataclass(frozen=True)
class DatasetDownloadCli:
    downloaders: dict[str, Ros1DatasetDownloader | Ros2DatasetDownloader]

    def parse_args(self, argv: Sequence[str] | None = None) -> argparse.Namespace:
        parser = argparse.ArgumentParser()
        parser.add_argument("target", choices=sorted(self.downloaders))
        parser.add_argument(
            "--data-dir",
            type=Path,
            default=Path(os.environ.get("SLAM_DATASET_DIR", "~/data")).expanduser(),
        )
        return parser.parse_args(argv)

    def run(self, argv: Sequence[str] | None = None) -> int:
        args = self.parse_args(argv)
        data_dir = args.data_dir.expanduser().resolve()
        data_dir.mkdir(parents=True, exist_ok=True)

        try:
            self.downloaders[args.target].download(data_dir)
        except Exception as exc:
            print(f"Dataset download failed: {exc}", file=sys.stderr)
            return 1

        return 0


def make_cli() -> DatasetDownloadCli:
    repository = HuggingFaceRepository(
        repo_id=REPO_ID,
        repo_type=REPO_TYPE,
        revision=REVISION,
    )
    copier = FileCopier()
    return DatasetDownloadCli(
        downloaders={
            "ros1": Ros1DatasetDownloader(repository=repository, copier=copier),
            "ros2": Ros2DatasetDownloader(repository=repository, copier=copier),
        }
    )


def parse_args() -> argparse.Namespace:
    return make_cli().parse_args()


def main() -> int:
    return make_cli().run()


if __name__ == "__main__":
    raise SystemExit(main())
