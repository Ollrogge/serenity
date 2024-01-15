/*
 * Copyright (c) 2020-2024, the SerenityOS developers.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <AK/AllOf.h>
#include <AK/CharacterTypes.h>
#include <AK/StdLibExtras.h>
#include <AK/String.h>
#include <AK/StringUtils.h>
#include <LibCore/ArgsParser.h>
#include <LibCore/File.h>
#include <LibCore/System.h>
#include <LibMain/Main.h>

static constexpr size_t BYTES_PER_LINE_HEX = 16;
static constexpr size_t BYTES_PER_LINE_C = 12;
static constexpr size_t BYTES_PER_LINE_BITS = 6;
static constexpr size_t BYTES_PER_LINE_PLAIN_HEX = 30;

static constexpr size_t GROUP_SIZE_HEX = 2;
static constexpr size_t GROUP_SIZE_HEX_LITTLE_ENDIAN = 4;
static constexpr size_t GROUP_SIZE_BITS = 1;
static constexpr size_t GROUP_SIZE_PLAIN_HEX = 0;

enum class DisplayStyle {
    Hex,
    PlainHex,
    HexLittleEndian,
    CStyle,
    Bits
};

static void print_ascii(Bytes line)
{
    for (auto const& byte : line) {
        if (is_ascii_printable(byte)) {
            putchar(byte);
        } else {
            putchar('.');
        }
    }
}

static void print_line_hex(Bytes line, size_t line_length_config, size_t group_size, bool uppercase)
{
    for (size_t i = 0; i < line_length_config; ++i) {
        if (i < line.size()) {
            if (uppercase) {
                out("{:02X}", line[i]);
            } else {

                out("{:02x}", line[i]);
            }
        } else {
            out("  ");
        }

        if (group_size != 0 && (i + 1) % group_size == 0) {
            out(" ");
        }
    }

    out(" ");
    print_ascii(line);
    putchar('\n');
}

static void print_line_little_endian_hex(Bytes line, size_t line_length_config, size_t group_size, bool uppercase)
{
    if (group_size == 1) {
        print_line_hex(line, line_length_config, group_size, uppercase);
        return;
    }

    if (group_size == 0 || group_size > BYTES_PER_LINE_HEX) {
        group_size = BYTES_PER_LINE_HEX;
    }

    for (size_t i = 0; i < line_length_config; i += group_size) {
        if (i < line.size()) {
            size_t size = i + group_size < line.size() ? group_size : line.size() - i;
            auto group = line.slice(i, size);
            if (size < group_size) {
                for (size_t i = 0; i < group_size - size; ++i) {
                    out("  ");
                }
            }
            for (ssize_t i = group.size() - 1; i >= 0; --i) {
                if (uppercase) {
                    out("{:02X}", group[i]);
                } else {

                    out("{:02x}", group[i]);
                }
            }
        } else {
            for (size_t i = 0; i < group_size; ++i) {
                out("  ");
            }
        }

        out(" ");
    }

    out(" ");
    print_ascii(line);
    putchar('\n');
}

static void print_line_bits(Bytes line, size_t line_length_config, size_t group_size)
{
    auto print_byte = [](u8 byte) {
        for (ssize_t i = 7; i >= 0; --i) {
            out("{}", (byte >> i) & 1 ? '1' : '0');
        }
    };

    for (size_t i = 0; i < line_length_config; ++i) {
        if (i < line.size()) {
            print_byte(line[i]);
        } else {
            out("         ");
        }

        if (group_size > 0 && (i + 1) % group_size == 0) {
            out(" ");
        }
    }

    out(" ");
    print_ascii(line);
    putchar('\n');
}

static void print_line_c_style(Bytes line)
{
    out("  ");
    for (size_t i = 0; i < line.size() - 1; ++i) {
        out("0x{:02x}, ", line[i]);
    }
    out("0x{:02x}", line[line.size() - 1]);
    putchar('\n');
}

static ErrorOr<String> path_to_variable_name(StringView path)
{
    auto work = path.to_byte_string();

    work = work.replace("."sv, "_"sv, ReplaceMode::All);
    work = work.replace("/"sv, "_"sv, ReplaceMode::All);

    return TRY(String::from_byte_string(work));
}

ErrorOr<int> serenity_main(Main::Arguments args)
{
    // todo: pledge

    Core::ArgsParser args_parser;
    StringView path;
    bool autoskip = false;
    bool c_include_file_style = false;
    bool capitalize_c_include_file_style = false;
    bool binary_digit_formatting = false;
    bool little_endian_hexdump = false;
    bool offset_in_decimal = false;
    bool plain_hexdump_style = false;
    bool uppercase_hex = false;
    Optional<size_t> line_length_option;
    Optional<size_t> group_size_option;
    Optional<size_t> max_bytes;
    Optional<size_t> position_offset;
    Optional<off_t> seek_to;
    String c_include_file_style_variable_name;
    StringView colorize_output_option;

    args_parser.add_positional_argument(path, "Input file", "input", Core::ArgsParser::Required::No);
    args_parser.add_option(autoskip, "'*' replaces nul-lines.", "autoskip", 'a');
    args_parser.add_option(binary_digit_formatting, "Binary digit formatting", "bits", 'b');
    args_parser.add_option(capitalize_c_include_file_style, "Capitalize C include file style (-i).", "capitalize", 'C');
    args_parser.add_option(line_length_option, "Amount of bytes shown per line (max 256)", "cols", 'c', "cols");
    args_parser.add_option(offset_in_decimal, "Show offset in decimal instead of hex", "decimal", 'd');
    args_parser.add_option(little_endian_hexdump, "Little-endian hex dump", nullptr, 'e');
    args_parser.add_option(group_size_option, "Separate the output of every n bytes", "groupsize", 'g', "n");
    args_parser.add_option(c_include_file_style, "Output in C include file style.", "include", 'i');
    args_parser.add_option(max_bytes, "Truncate to fixed number of bytes", "len", 'l', "bytes");
    args_parser.add_option(c_include_file_style_variable_name, "Set variable name used in C include ouput (-i)", "name", 'n', "include_style");
    args_parser.add_option(position_offset, "Add offset to displayed file position", nullptr, 'o', "offset");
    args_parser.add_option(plain_hexdump_style, "Output in plain hex dump style", "plain", 'p');
    args_parser.add_option(seek_to, "Seek to a byte offset", "seek", 's', "[-]offset");
    args_parser.add_option(uppercase_hex, "Use upper case hex letters", nullptr, 'u');
    args_parser.add_option(colorize_output_option, "Colorize output", nullptr, 'R', "when");

    args_parser.parse(args);

    auto file = TRY(Core::File::open_file_or_standard_stream(path, Core::File::OpenMode::Read));
    auto file_size = TRY(file->size());

    auto display_style = DisplayStyle::Hex;
    size_t line_length_config = BYTES_PER_LINE_HEX;
    size_t group_size = GROUP_SIZE_HEX;

    // todo: put states that can appear together in an if, else if block
    // todo: how to handle invalid combinations ?
    // lazy approach ?

    if (c_include_file_style) {
        line_length_config = BYTES_PER_LINE_C;
        display_style = DisplayStyle::CStyle;

        if (c_include_file_style_variable_name.is_empty()) {
            c_include_file_style_variable_name = TRY(path_to_variable_name(path));
        }

        if (capitalize_c_include_file_style) {
            c_include_file_style_variable_name = TRY(c_include_file_style_variable_name.to_uppercase());
        }

        outln("u8 {}[] = {{", c_include_file_style_variable_name);
    }

    if (little_endian_hexdump) {
        display_style = DisplayStyle::HexLittleEndian;
        group_size = GROUP_SIZE_HEX_LITTLE_ENDIAN;
    }

    if (plain_hexdump_style) {
        display_style = DisplayStyle::PlainHex;
        line_length_config = BYTES_PER_LINE_PLAIN_HEX;
        group_size = GROUP_SIZE_PLAIN_HEX;
    }

    if (line_length_option.has_value() && line_length_option.value() > 0) {
        line_length_config = line_length_option.value();

        if (line_length_config > 256) {
            outln("Invalid number of columns (max is 256).");
            return 1;
        }
    }

    if (binary_digit_formatting) {
        if (c_include_file_style || plain_hexdump_style) {
            outln("-ps and -i are incompatible with -b");
            return 1;
        }

        display_style = DisplayStyle::Bits;
        group_size = GROUP_SIZE_BITS;
        line_length_config = BYTES_PER_LINE_BITS;
    }

    if (group_size_option.has_value()) {
        group_size = group_size_option.value();

        if (little_endian_hexdump) {
            if (group_size != 0 && !is_power_of_two(group_size)) {
                outln("Group size must be a power of 2 with -e");
                return 1;
            }
        }
    }

    // TODO: colorize output
    if (!colorize_output_option.is_null()) {
        outln("Colorizing output is not supported");
    }

    Array<u8, BUFSIZ> contents;
    Bytes bytes;
    size_t total_bytes_read = 0x0;
    const size_t max_read_size = contents.size() - (contents.size() % line_length_config);
    bool is_input_remaining = true;

    // TODO: seek relative to current stdin file position
    if (seek_to.has_value()) {
        auto offset = seek_to.value();
        total_bytes_read = offset < 0 ? file_size + offset : offset;
        TRY(file->seek(total_bytes_read, SeekMode::SetPosition));
    }

    while (is_input_remaining) {
        auto bytes_to_read = max_read_size - bytes.size();

        bytes = contents.span().slice(0, bytes_to_read);
        bytes = TRY(file->read_some(bytes));

        if (bytes.size() < bytes_to_read) {
            is_input_remaining = false;
        }

        while (bytes.size() > 0) {
            auto line_length = bytes.size() > line_length_config ? line_length_config : bytes.size();

            if (max_bytes.has_value()) {
                auto bytes_remaining = max_bytes.value() - total_bytes_read;
                if (bytes_remaining < line_length) {
                    line_length = bytes_remaining;
                }
            }

            auto current_line = bytes.slice(0, line_length);
            bytes = bytes.slice(line_length);

            if (autoskip && all_of(bytes, [](auto& b) { return b == 0x0; })) {
                outln("*");
                continue;
            }

            if (display_style != DisplayStyle::CStyle && display_style != DisplayStyle::PlainHex) {
                auto offset = 0;
                if (position_offset.has_value()) {
                    offset = position_offset.value();
                }

                if (offset_in_decimal) {
                    out("{:08}: ", total_bytes_read + offset);
                } else {
                    out("{:08x}: ", total_bytes_read + offset);
                }
            }

            switch (display_style) {
            case DisplayStyle::PlainHex:
            case DisplayStyle::Hex:
                print_line_hex(current_line, line_length_config, group_size, uppercase_hex);
                break;
            case DisplayStyle::HexLittleEndian:
                print_line_little_endian_hex(current_line, line_length_config, group_size, uppercase_hex);
                break;
            case DisplayStyle::Bits:
                print_line_bits(current_line, line_length_config, group_size);
                break;
            case DisplayStyle::CStyle:
                print_line_c_style(current_line);
                break;
            }

            total_bytes_read += line_length_config;

            if (max_bytes.has_value() && total_bytes_read >= max_bytes.value()) {
                is_input_remaining = false;
                break;
            }
        }
    }

    if (display_style == DisplayStyle::CStyle) {
        outln("}};");
        outln("size_t {}_len = {};", c_include_file_style_variable_name, total_bytes_read);
    }

    return 0;
}
