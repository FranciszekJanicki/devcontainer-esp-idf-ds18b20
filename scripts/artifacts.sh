#!/usr/bin/env bash


GITLAB_URL="https://gitlab.com"
GITLAB_ARTIFACT_TOKEN=$(cat gitlab-token.txt)
project_id="56202787"
branch="v1.3.0"         # TAG or branch name, eg. main, develop...
job="compile-nanopb"    # compile-nanopb, compile-python, compile-js-ts

output_file="/tmp/artifacts.zip"
component_dir="components/putrl_proto"

curl --location --header "PRIVATE-TOKEN: $GITLAB_ARTIFACT_TOKEN" \
    -o $output_file \
    "$GITLAB_URL/api/v4/projects/$project_id/jobs/artifacts/$branch/download?job=$job&artifact_format=zip&artifact_type=archive"

unzip $output_file -d "/tmp"

rm -rf "$component_dir/nanopb" "$component_dir/*.c" "$component_dir/*.h"

cp /tmp/proto-files/*.c "$component_dir"
cp /tmp/proto-files/*.h "$component_dir"
mv "/tmp/proto-files/nanopb" "$component_dir"

rm $output_file
rm -rf "/tmp/proto-files"
