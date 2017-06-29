#...............................................................................
#
#  This file is part of the Doxyrest toolkit.
#
#  Doxyrest is distributed under the MIT license.
#  For details see accompanying license.txt file,
#  the public copy of which is also available at:
#  http://tibbo.com/downloads/archive/doxyrest/license.txt
#
#...............................................................................
cmake_minimum_required(VERSION 2.8.11)
set (DOC_NAME "star2 documentation")

set (STAR2_VERSION_MAJOR 2)
set (STAR2_VERSION_MINOR 0)
set (STAR2_VERSION_REVISION 0)

set (STAR2_VERSION_COMPANY "Kristopher Wehage")
set (STAR2_VERSION_YEARS   "2015-2017")

set (STATIC_RST_DIR ${CMAKE_CURRENT_LIST_DIR}/sphinx)

set (XML_DIR  ${CMAKE_CURRENT_LIST_DIR}/../build/xml)
set (RST_DIR  ${CMAKE_CURRENT_BINARY_DIR}/sphinx)
set (TMP_DIR  ${CMAKE_CURRENT_BINARY_DIR}/tmp)
set (HTML_DIR ${CMAKE_CURRENT_LIST_DIR}/build/html)
set (DOXYREST_FRAME_DIR ${CMAKE_CURRENT_BINARY_DIR}/sphinx/frame)

#. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

set (
	DOXYREST_CMD_LINE
	${XML_DIR}/index.xml
	-D g_hasNewLineAfterReturnType
	-D g_nameDelimiter=::
	-D g_escapeAsterisks
	-D g_verbatimToCodeBlock
	-o ${RST_DIR}/index.rst
	-F ${DOXYREST_FRAME_DIR}
	-f c_index.rst.in
	)

set (
	SPHINX_HTML_CMD_LINE
	-b html
	-d ${TMP_DIR}/sphinx-doctrees
	-c ${CMAKE_CURRENT_BINARY_DIR}
	${RST_DIR}
	${HTML_DIR}
	)

#. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

set (
	CMD_LINE_VAR_LIST

	DOXYREST_CMD_LINE
	SPHINX_HTML_CMD_LINE
	)

foreach (_VAR ${CMD_LINE_VAR_LIST})
	string (
		REPLACE
		";" " "
		${_VAR}
		"${${_VAR}}"
		)
endforeach ()

#. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

configure_file (
	sphinx/conf.py.in
	sphinx/conf.py
	)


add_test (
	NAME STAR2
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMAND ./build-rst.sh
	)

#...............................................................................
