project('removedirtvs', 'cpp',
  version: '1.0',
  default_options: ['cpp_std=c++11', 'buildtype=release', 'warning_level=2'],
  meson_version: '>=0.51.0'
)

sources = [
  'RemoveDirt.cpp',
  'RestoreMotionBlocks.cpp',
  'SCSelect.cpp',
  'DupBlocks.cpp',
  'shared.cpp',
]

vapoursynth_dep = dependency('vapoursynth', required: false)

if vapoursynth_dep.found()
  deps = [vapoursynth_dep]
  vapoursynth_plugindir = vapoursynth_dep.get_variable(pkgconfig: 'libdir') / 'vapoursynth'
else
  deps = []
  warning('VapourSynth dependency not found via pkg-config, building without it')
  vapoursynth_plugindir = get_option('libdir') / 'vapoursynth'
endif

shared_module('removedirtvs',
  sources,
  dependencies: deps,
  install: true,
  install_dir: vapoursynth_plugindir,
  gnu_symbol_visibility: 'hidden',
  name_prefix: 'lib'
)
