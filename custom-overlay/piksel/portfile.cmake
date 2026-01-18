vcpkg_check_linkage(ONLY_STATIC_LIBRARY)

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO v4m3rrr/piksel
    REF "${VERSION}"
    SHA512 6f33f0c98101e1250e2d8f4a4e5555a69e69f9a7567dd96e59541e548c54ae9015fe87d21ddc74736c6cc7b0c78337794de392ea76713ad7a87232a3c45d19f3
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
