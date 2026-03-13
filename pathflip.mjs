// pathflip - generates left/right mirrored variants of auto paths
// node pathflip.mjs -i comp-bot/src/main/java/frc/robot/autos/auto_state_machines/RightCircleSoMAuto.java

import assert from "node:assert/strict";
import fs from "node:fs/promises";
import { parseArgs } from "node:util";

const args = parseArgs({
  options: {
    input: { type: "string", short: "i" },
  },
});

const { input: inputPath } = args.values;

assert(inputPath, "input is required");

const inputContents = await fs.readFile(inputPath, "utf8");

const FIELD_HEIGHT = 8.069;
const POSE_2D_REGEXP = /new\s+Pose2d\(([\d.]+),\s*([\d.]+)/g;
const ROTATION_2D_REGEXP = /Rotation2d\.fromDegrees\((-?[\d.]+)\)/g;
const ROTATION_CONSTANT_REGEXP = /Rotation2d\.(kZero|k180deg|kCW_90deg|kCCW_90deg)/g;

function stupidRound(value, precision) {
  return Number(value.toFixed(precision));
}

function verticalFlip(x, y) {
  const xAxis = FIELD_HEIGHT / 2;
  return {
    x,
    y: xAxis - (y - xAxis),
  };
}

const TWO_PI = 2 * Math.PI;

/** Normalizes an angle to be within (-pi, pi]. */
function angleModulusRadians(angle) {
  const result = angle - TWO_PI * Math.floor((angle + Math.PI) / TWO_PI);

  if (result === -Math.PI) {
    return Math.PI;
  }

  return result;
}

function angleModulusDegrees(angle) {
  return angleModulusRadians((angle / 180) * Math.PI) * (180 / Math.PI);
}

function transformRotationVertical(rotationDeg) {
  return angleModulusDegrees(-rotationDeg);
}

const ROTATION_CONSTANT_FLIPS = {
  kZero: "kZero",
  k180deg: "k180deg",
  kCW_90deg: "kCCW_90deg",
  kCCW_90deg: "kCW_90deg",
};

/** @param {string} text */
function transform(text) {
  return multiReplace(
    text
      .replaceAll(POSE_2D_REGEXP, (_, xString, yString) => {
        const { x, y } = verticalFlip(Number(xString), Number(yString));
        return `new Pose2d(${stupidRound(x, 3)}, ${stupidRound(y, 3)}`;
      })
      .replaceAll(ROTATION_2D_REGEXP, (_, degreesString) => {
        return `Rotation2d.fromDegrees(${stupidRound(
          transformRotationVertical(Number(degreesString)),
          3
        )})`;
      })
      .replaceAll(ROTATION_CONSTANT_REGEXP, (_, constant) => {
        return `Rotation2d.${ROTATION_CONSTANT_FLIPS[constant]}`;
      }),
    {
      OUTPOST: "DEPOT",
      Outpost: "Depot",
      outpost: "depot",
      DEPOT: "OUTPOST",
      Depot: "Outpost",
      depot: "outpost",
      Right: "Left",
      right: "left",
      RIGHT: "LEFT",
      Left: "Right",
      left: "right",
      LEFT: "RIGHT",
    }
  );
}

const outputPath = multiReplace(inputPath, {
  Right: "Left",
  right: "left",
  RIGHT: "LEFT",
  Left: "Right",
  left: "right",
  LEFT: "RIGHT",
});

assert(outputPath !== inputPath, "could not determine output path - input file name must contain Right/Left");

await fs.writeFile(outputPath, transform(inputContents));
console.log(`Wrote ${outputPath}`);

function multiReplace(string, replacements) {
  const replacementsIterable = Object.entries(replacements);
  let result = "";
  let index = 0;

  while (index < string.length) {
    foundReplace: {
      for (const [searchValue, replaceValue] of replacementsIterable) {
        if (string.slice(index).startsWith(searchValue)) {
          result += replaceValue;
          index += searchValue.length;
          break foundReplace;
        }
      }

      result += string[index++];
    }
  }

  return result;
}
