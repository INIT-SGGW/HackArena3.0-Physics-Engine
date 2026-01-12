vcpkg_check_linkage(ONLY_STATIC_LIBRARY)

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO v4m3rrr/piksel
    REF "${VERSION}"
    SHA512 3e6daa1587e006b8d9bc5cdf6284aef57fc689ef0ddae445e41cd2354f4a11e461abe3ce63c365d9b52777d204a7e3bc91ee472bacbcc85f6c9e3c08a791e965
    HEAD_REF main
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(
    PACKAGE_NAME "piksel"
    CONFIG_PATH "lib/cmake/piksel"
)

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
