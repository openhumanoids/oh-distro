#!/bin/bash

ssh atlas2 -t 'bash -ic "sudo halt"'
ssh atlas1 -t 'bash -ic "sudo halt"'
ssh atlas0 -t 'bash -ic "sudo halt"'

