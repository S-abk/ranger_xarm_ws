#!/bin/bash
# Teleport the base back to a fixed start pose so each rough-ground trial
# begins on the same patch of bumps instead of wherever the last one ended.
gz service -s /world/empty_ground/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "ranger_xarm", position: {x: 0, y: 0, z: 0.15}, orientation: {w: 1}' 2>&1 | tail -1
