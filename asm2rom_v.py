#!/usr/bin/env python3
# Copyright (c) 2018, Itzik Kotler
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import argparse
import re
import enum

####################
# Global Variables #
####################

__version__ = "0.0.1"


###########
# Classes #
###########

class Assembler:

    def __init__(self, assembly_program):
        self._input = assembly_program
        self._output = []
        self._context = {}

    def assemble(self):
        self._context['PROGRAM_COUNTER'] = 0
        self._context['CURRENT_LINE'] = 0
        self._context['LABELS'] = {}

        # Iterate Program Source
        for raw_line in self._input:

            self._context['CURRENT_LINE'] += 1

            # Remove Whitespaces
            processed_line = raw_line.strip()

            # Empty Line? Skip!
            if not processed_line:
                continue

            result = None
            instructions_cntr_incr = 0

            for (action_name, action_type, action_arg, regex) in self.ACTIONS:
                result = regex.match(processed_line)
                if result:
                    # Do it, cmd-like Python Module Style
                    action_fcn_name = "do_"
                    action_fcn_args = result.groups()[1:]
                    if action_type == self._AssemblerActions.AS_IS:
                        # i.e., `do_lda(...)' for `LDA $1'
                        action_fcn_name += result.groupdict()['NAME']
                    elif action_type == self._AssemblerActions.ADD_PREFIX:
                        # i.e., `do_label('start', ...)' for `START:'
                        action_fcn_name += action_arg + '_' + result.groupdict()['NAME']
                    elif action_type == self._AssemblerActions.OVERRIDE:
                        # i.e., `do_directive_org' for `.org 0x1000'
                        action_fcn_name += action_arg
                        action_fcn_args = result.groups()
                    else:
                        raise Exception("Internal Error! Found {} for {} but AssemblerAction ID: {} is Unknown!".format(result.string, regex, action_type))

                    # Invoke Action Function
                    try:
                        action_fcn = getattr(self, action_fcn_name.lower())
                        (instructions_cntr_incr, binary_code) = self.post_action(*action_fcn(*action_fcn_args))
                        self._output.append({
                                'ACTION_OUTPUT': binary_code,
                                'ACTION_LINE': processed_line,
                                'ACTION_NAME': action_name
                            })
                    except AttributeError:
                        raise Exception("Unimplmented Method for: {} (LINE #{}: {})".format(action_fcn_name, self._context['CURRENT_LINE'], processed_line))

                    # We're not greedy!
                    break

            # Can't Match? Syntax Error!
            if result is None:
                raise Exception("Syntax Error: {}".format(processed_line))

            # Increase Counters
            self._context['PROGRAM_COUNTER'] += instructions_cntr_incr

    def verilog_output(self):
        _buffer = []
        _tot_instructions = 0

        # Iterate Events
        for event in self._output:

            # Output Label as Verilog Comment
            if event['ACTION_NAME'] == "LABEL":
                _buffer.append("// {}".format(event['ACTION_LINE']))

            # Output Instruction as Verilog Array
            if event['ACTION_NAME'] == "INSTRUCTION":
                instr = event['ACTION_OUTPUT']

                _buffer.append("rom[{:d}] = {:d}'b{:{fill}{width}b}; // {}".format(
                    _tot_instructions,
                    len(instr)*8,
                    ord(instr),
                    event['ACTION_LINE'],
                    fill=0, width=len(instr)*8))

                _tot_instructions += 1

        return "\n".join(_buffer) + "\n"

    def _imm_to_bin(self, imm_as_ascii):
        # i.e., 0xF -> '1111'
        return int(imm_as_ascii, 16)

    def post_action(self, action_instr_incr, action_output):
        return (action_instr_incr, action_output)

    # Action Enum
    class _AssemblerActions(enum.Enum):
        AS_IS = 0
        ADD_PREFIX = 1
        OVERRIDE = 2

    #  Actions Type, Action Argument, RegEx (Parser)
    ACTIONS = [
        ("COMMENT", _AssemblerActions.OVERRIDE, "comment", re.compile(r"#.*")),
        ("DIRECTIVE", _AssemblerActions.ADD_PREFIX, "directive", re.compile(r"\.(?P<NAME>[a-zA-Z0-9]+)[ \t]+(?P<VALUE>.*)")),
        ("LABEL", _AssemblerActions.OVERRIDE, "label", re.compile(r"(?P<NAME>[a-zA-Z0-9_].*):")),
        ("INSTRUCTION", _AssemblerActions.AS_IS, None, re.compile(r"(?P<NAME>[a-zA-Z0-9]+)[ \t]+(?P<VALUE>.*)"))
    ]


class PinkyAssembler(Assembler):
    def post_action(self, action_instr_incr, action_output):
        try:
            return (action_instr_incr, action_output.to_bytes(1, byteorder='big'))
        except Exception:
            return (action_instr_incr, action_output)

    def _do_alu(self, alu_opcode, regs):
        (dst_reg, src_reg) = regs.split(',')
        return (1, int("{:03b}{:b}{:b}{:03b}".format(
            0b011,                      # ALU OPCODE
            self._reg_to_bin(dst_reg),  # DST REGISTER (OPERAND)
            self._reg_to_bin(src_reg),  # SRC REGISTER (OPERAND)
            alu_opcode                  # ADC OPCODE (SPECIFIC ALU OPCODE)
            ), 2))

    def _do_store(self, reg, data):
        return (1, int("{:03b}{:b}{:04b}".format(
            0b010,
            reg,
            int(data[1:], 16)
            ), 2))

    def _do_branch(self, branch_opcode, distance):
        return (1, int("{:03b}{:02b}{:03b}".format(
            0b100,                      # BRANCH OPCODE
            branch_opcode,              # JUMP NO CARRY (SPECIFIC BRANCH OPCODE)
            distance                    # IMM3 (UNSIGNED/SIGNED DEPENDS ON BRANCH OPCODE)
            ), 2))

    def do_lda(self, *args, **kwargs):
        # e.g., 'LDA $0xF'
        return self._do_store(0, args[0])

    def do_ldb(self, *args, **kwargs):
        # e.g., 'LDB $0xF'
        return self._do_store(1, args[0])

    def do_comment(self, *args, **kwargs):
        return (0, None)

    def do_label(self, *args, **kwargs):
        if args[0] in self._context['LABELS'] is True:
            # Duplicate
            raise Exception("Duplicate for Label: {}".format(args[0]))
        else:
            # Add Label (eq to Virtual CURRENT_IP)
            self._context['LABELS'][args[0]] = self._context['PROGRAM_COUNTER']

        return (0, None)

    def _immediate(self, data):
        return int(data.strip()[1:], 16)

    def _reg_to_bin(self, reg):
        current_reg = reg.upper().strip()
        if current_reg == 'A':
            return 0
        elif current_reg == 'B':
            return 1
        else:
            raise Exception("Unknown Register: {}".format(reg))

    def do_or(self, *args, **kwargs):
        # e.g., 'OR A, A'
        return self._do_alu(0b010, args[0])

    def do_add(self, *args, **kwargs):
        # e.g., 'ADC A,B'
        return self._do_alu(0b100, args[0])

    def do_jnc(self, *args, **kwargs):
        # e.g., 'JNC _START'
        distance = (self._context['LABELS'][args[0]] - (self._context['PROGRAM_COUNTER']))
        if (distance < self.IMM3_MIN or distance > self.IMM3_MAX):
            raise Exception("Realtive Distance {} Is Too Far!".format(distance))
        return self._do_branch(0b00, distance & 0x7)

    def do_jne(self, *args, **kwargs):
        # e.g., 'JNC _START'
        distance = (self._context['LABELS'][args[0]] - (self._context['PROGRAM_COUNTER']))
        if (distance < self.IMM3_MIN or distance > self.IMM3_MAX):
            raise Exception("Realtive Distance {} Is Too Far!".format(distance))
        return self._do_branch(0b01, distance & 0x7)

    def do_out(self, *args, **kwargs):
        (dst_port, data) = args[0].split(',')

        # e.g., 'OUT $1, $1'
        return (1, int("{:03b}{:b}{:02b}{:02b}".format(
            0b111,                          # IN/OUT OPCODE
            1,                              # OUT (I/O SPECIFIC OPCODE)
            int(dst_port.strip()[1:], 16),  # PORT NUMBER (OPERNAD)
            int(data.strip()[1:], 16)       # DATA (OPERAND)
            ), 2))

    def do_sub(self, *args, **kwargs):
        # e.g., 'SBB A, B'
        return self._do_alu(0b101, args[0])

    def do_jmp(self, *args, **kwargs):
        # e.g., 'JMP _START'
        try:
            absolute_addr = self._context['LABELS'][args[0]]
        except KeyError:
            absolute_addr = int(args[0][1:], 16)
        return self._do_branch(0b11, absolute_addr)

    def do_cmp(self, *args, **kwargs):
        (dst_reg, data) = args[0].split(',')

        # e.g., 'CMP A, $7'
        return (1, int("{:03b}{:01b}{:04b}".format(
            0b110,
            self._reg_to_bin(dst_reg),
            self._immediate(data)
            ), 2))

    IMM3_MIN = (-2**3)+1
    IMM3_MAX = (2**3)-1


#############
# Functions #
#############

def main(args):
    args_parser = argparse.ArgumentParser(prog='Pinky Assembler', description="Assembler for Pinky 8-bit Microprocessor")
    args_parser.add_argument("-i", "--infile", metavar="ASM_FILE", type=argparse.FileType('r'), default=sys.stdin, help="Reading Pinky 8-bit Assembly Program (default: STDIN)")
    args_parser.add_argument("-o", "--outfile", metavar="VERILOG_FILE", type=argparse.FileType('w+b'), default=sys.stdout, help="Writing Pinky 8-bit Verilog ROM (default: \"STDOUT\")")
    args_parser.add_argument("-v", "--verbose", action="count", default=0, help="Increase Output Verbosity (Repeat up to two times)")
    args_parser.add_argument('--version', action='version', version="%(prog)s v" + __version__)
    args_parser = args_parser.parse_args(args)

    assembler = PinkyAssembler(args_parser.infile)

    assembler.assemble()

    args_parser.outfile.write(assembler.verilog_output())


###############
# Entry Point #
###############

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
