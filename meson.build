#
# Copyright (C) 2019, 2022  T+A elektroakustik GmbH & Co. KG
#
# This file is part of DCPSPI.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
# MA  02110-1301, USA.
#

project('DCPSPI', ['cpp', 'c'], license: 'GPL2+',
    version: '3.2',
    default_options: ['warning_level=3', 'cpp_std=c++14', 'buildtype=plain'],
)

# Configuration header file
config_data = configuration_data()
config_data.set('PACKAGE', 'dcpspi')
config_data.set('PACKAGE_NAME', meson.project_name())
config_data.set('PACKAGE_VERSION', meson.project_version())
config_data.set('abs_srcdir', meson.current_source_dir())
config_data.set('abs_builddir', meson.build_root())
config_data.set('bindir', get_option('prefix') / get_option('bindir'))
configure_file(input: 'config.h.meson', output: 'config.h', configuration: config_data)

add_project_arguments('-DHAVE_CONFIG_H', language: ['cpp', 'c'])

# Doxygen documentation
doxygen = find_program('doxygen', required: false)

if doxygen.found()
    doxyfile = configure_file(input: 'Doxyfile.in', output: 'Doxyfile', configuration: config_data)
    custom_target('doxygen', output: 'doxygen.stamp', input: doxyfile, command: doxygen)
endif

# README file
markdown = find_program('markdown', required: false)

if markdown.found()
    custom_target('documentation',
        output: 'README.html', input: 'README.md',
        command: [markdown, '-o', '@OUTPUT@', '@INPUT@'],
        install: true, install_dir: get_option('datadir') / 'doc/dcpspi')
endif

# Software version information extracted from Git repository
versioninfo = custom_target(
    'version information file',
    input: configure_file(input: 'versioninfo.cache.in', output: 'versioninfo.cache', copy: true),
    output: 'versioninfo.h',
    command: [
        find_program('autorevision'),
        '-C', meson.current_source_dir(),
        '-t', 'h',
        '-o', 'versioninfo.cache'
    ],
    capture: true,
)

# Libraries for production code and unit tests
spi_lib = static_library('libspi', 'spi.c')
dcpspi_lib = static_library('libdcpspi', 'dcpspi_process.c')
statistics_lib = static_library('libstatistics', 'statistics.c')

# The final executable
executable(
    'dcpspi',
    [
        'dcpspi.c', 'os.c', 'messages.c', 'messages_signal.c', 'named_pipe.c',
        'hexdump.c', 'gpio.c', 'spi_hw.c', versioninfo,
    ],
    link_with: [spi_lib, dcpspi_lib, statistics_lib],
    install: true,
)

# Unit tests
subdir('tests')
