add_subdirectory(src/utVisionComponents)
add_subdirectory(src/utVisionIOComponents)
ut_install_utql_patterns()

# install custom files
file(GLOB _doc_files LIST_DIRECTORIES false "doc/marker/*" "doc/marker/*/*")
foreach(pfile ${_doc_files})
  string(REGEX REPLACE "^.*/doc/" "" pfile2 "${pfile}")
  GET_FILENAME_COMPONENT(fpath ${pfile2} PATH)
  IF(fpath)
	install(FILES ${pfile} DESTINATION "${UBITRACK_DOC_INSTALL_DIRECTORY}/${fpath}" COMPONENT doc)
  ELSE(fpath)
    install(FILES ${pfile} DESTINATION "${UBITRACK_DOC_INSTALL_DIRECTORY}" COMPONENT doc)
  ENDIF(fpath)
endforeach()

add_subdirectory(apps/MarkerBundle)
add_subdirectory(apps/CornerFinder)