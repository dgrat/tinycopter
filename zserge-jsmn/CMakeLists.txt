cmake_minimum_required(VERSION 2.8)
project(RPiCopter)

SET( zserge-jsmn
  jsmn
)

ADD_LIBRARY ( jsmn STATIC ${zserge-jsmn} )