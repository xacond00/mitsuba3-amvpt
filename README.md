<!-- <img src="https://github.com/mitsuba-renderer/mitsuba3/raw/master/docs/images/logo_plain.png" width="120" height="120" alt="Mitsuba logo"> -->

<img src="https://raw.githubusercontent.com/mitsuba-renderer/mitsuba-data/master/docs/images/banners/banner_01.jpg"
alt="Mitsuba banner">

# Mitsuba Renderer 3 - Unofficial AMVPT Extension
## Diplomová práce
### Bc. Ondřej Áč (xacond00)
Compilation and execution guide:
```shell
mkdir build && cd build
cmake [-GNinja] .. # Ninja generator is recommended
```
Check that `build/mitsuba.conf` contains:
```python
"enabled": [
        "scalar_rgb", "cuda_rgb", "llvm_rgb", "llvm_ad_spectral"
]
```
If your gpu doesn't support CUDA, delete `cuda_rgb`.
```shell
[make|ninja] # Run generator and go get a coffee
./mitsuba # Run GUI
./mitsuba -m [cuda_rgb|llvm_rgb] ../xxx/xxx/scene.xml # Render a specific scene
```
For detailed instructions see documentation. 

# Credits:
```bibtex
@software{Mitsuba3,
    title = {Mitsuba 3 renderer},
    author = {Wenzel Jakob and Sébastien Speierer and Nicolas Roussel and Merlin Nimier-David and Delio Vicini and Tizian Zeltner and Baptiste Nicolet and Miguel Crespo and Vincent Leroy and Ziyi Zhang},
    note = {https://mitsuba-renderer.org},
    version = {3.1.1},
    year = 2022
}
```
