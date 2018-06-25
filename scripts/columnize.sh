#!/bin/bash
read str
echo "$str" | tr ' ' '\n' | clip.exe
