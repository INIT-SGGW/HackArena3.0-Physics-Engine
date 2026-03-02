vcpkg_check_linkage(ONLY_STATIC_LIBRARY)

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO v4m3rrr/piksel
    REF "${VERSION}"
    SHA512 9e73217e34dc51944b52d01282868eeafbbf89a25445e1a4c08f4d9e064f1de9286da4e09bf3d034612321c01b07ce933e60cb3b1159cfa82017ba08d48c2050
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
