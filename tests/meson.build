#
# Copyright (C) 2019  T+A elektroakustik GmbH & Co. KG
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

cutter_dep = dependency('cppcutter', required: false)
compiler = meson.get_compiler('cpp')

if not(compiler.has_header('cutter.h', dependencies: cutter_dep) and \
       compiler.has_header('cppcutter.h', dependencies: cutter_dep))
    subdir_done()
endif

cutter_wrap = find_program('cutter_wrap.sh')

cutter_wrap_args = [
    find_program('cutter').path(), find_program('xsltproc').path(),
    meson.current_build_dir(), meson.current_source_dir()
]

spi_tests = shared_module('test_spi',
    ['test_spi.cc', 'mock_os.cc', 'mock_messages.cc', 'mock_spi_hw.cc'],
    include_directories: ['..'],
    dependencies: cutter_dep,
    link_with: [spi_lib, statistics_lib],
)

test('SPI low level',
    cutter_wrap, args: [cutter_wrap_args, spi_tests.full_path()],
    depends: spi_tests
)

complete_tests = shared_module('test_complete',
    ['test_complete.cc', 'mock_os.cc', 'mock_messages.cc', 'mock_spi_hw.cc', 'mock_gpio.cc'],
    include_directories: ['..'],
    dependencies: cutter_dep,
    link_with: [dcpspi_lib, spi_lib, statistics_lib],
)

test('Complete transfers',
    cutter_wrap, args: [cutter_wrap_args, complete_tests.full_path()],
    depends: complete_tests
)

statistics_tests = shared_module('test_statistics',
    ['test_statistics.cc', 'mock_os.cc', 'mock_messages.cc'],
    include_directories: ['..'],
    dependencies: cutter_dep,
    link_with: statistics_lib,
)

test('Statistics',
    cutter_wrap, args: [cutter_wrap_args, statistics_tests.full_path()],
    depends: statistics_tests
)

