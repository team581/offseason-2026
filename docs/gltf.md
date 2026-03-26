# glTF

To compress glTF files, use `gltf-transform`

```sh
gltf-transform optimize input.glb compressed.glb --palette false --instance false --compress false
```

The reason for disabling the default options is explained below:
- Palette: reduces the colors in textures, which makes the model look wrong
- Instance: causes every part in the model to be equal size and stacked on top of each other
- Compress: AdvantageScope doesn't support meshoptimizer compressed glTF models, see https://github.com/Mechanical-Advantage/AdvantageScope/pull/492
