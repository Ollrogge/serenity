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

static void print_line_hex(Bytes line, size_t line_length, size_t group_size, bool uppercase)
{
    for (size_t i = 0; i < line_length; ++i) {
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

    out("  ");
    print_ascii(line);
    putchar('\n');
}

static void print_line_little_endian_hex(Bytes line, size_t line_length, size_t group_size, bool uppercase)
{
    if (group_size < 2) {
        print_line_hex(line, line_length, group_size, uppercase);
        return;
    }

    size_t padding = (line_length - line.size()) / group_size;
    line_length -= padding;

    for (size_t i = 0; i < line_length; i += group_size) {
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
        out(" ");
    }

    for (size_t i = 0; i < padding; ++i) {
        for (size_t i = 0; i < group_size; ++i) {
            out("  ");
        }
    }

    out("  ");
    print_ascii(line);
    putchar('\n');
}

static void print_line_bits(Bytes line, size_t line_length, size_t group_size)
{
    auto print_byte = [](u8 byte) {
        for (ssize_t i = 7; i >= 0; --i) {
            out("{}", (byte >> i) & 1 ? '1' : '0');
        }
    };

    for (size_t i = 0; i < line_length; ++i) {
        if (i < line.size()) {
            print_byte(line[i]);
        } else {
            out("         ");
        }

        if (group_size > 0 && (i + 1) % group_size == 0) {
            out(" ");
        }
    }

    out("  ");
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
    StringView seek_offset_setting;
    bool autoskip = false;
    bool c_include_file_style = false;
    bool capitalize_c_include_file_style = false;
    bool binary_digit_formatting = false;
    bool little_endian_hexdump = false;
    bool offset_in_decimal = false;
    bool plain_hexdump_style = false;
    bool uppercase_hex = false;
    Optional<size_t> line_length_setting;
    Optional<size_t> group_size_setting;
    Optional<size_t> read_limit_setting;
    Optional<size_t> added_file_offset_setting;
    String c_include_file_style_variable_name;

    args_parser.add_positional_argument(path, "Input file", "input", Core::ArgsParser::Required::No);
    args_parser.add_option(autoskip, "A single '*' replaces NUL-lines.", "autoskip", 'a');
    args_parser.add_option(c_include_file_style, "Output in C include file style.", "include", 'i');
    args_parser.add_option(capitalize_c_include_file_style, "Capitalize C include file style (-i).", "capitalize", 'C');
    args_parser.add_option(binary_digit_formatting, "Binary digit formatting", "bits", 'b');
    args_parser.add_option(plain_hexdump_style, "Plain hex dump style", "plain", 'p');
    args_parser.add_option(offset_in_decimal, "Show offset in decimal instead of hex", "decimal", 'd');
    args_parser.add_option(line_length_setting, "Octets per line", "cols", 'c', "cols");
    args_parser.add_option(little_endian_hexdump, "Little-endian hex dump", nullptr, 'e');
    args_parser.add_option(uppercase_hex, "Use upper case hexh letters", "uppercase", 'u');
    args_parser.add_option(group_size_setting, "Octet group size", "groupsize", 'g', "groupsize");
    args_parser.add_option(read_limit_setting, "Stop after writing <len> octets", "len", 'l', "octet_length");
    args_parser.add_option(c_include_file_style_variable_name, "Set the variable name used in C include ouput (-i)", "name", 'n', "include_style");
    args_parser.add_option(added_file_offset_setting, "Add <off> to displayed file offset", "offset", 'o', "offset");
    args_parser.add_option(seek_offset_setting, "start at <seek> bytes absolute infile offset", "[-]seek", 's', "seek");
    args_parser.parse(args);

    outln("Test: {} {}", c_include_file_style_variable_name, c_include_file_style_variable_name.is_empty());

    // todo: TRY vs MUST

    auto file = TRY(Core::File::open_file_or_standard_stream(path, Core::File::OpenMode::Read));
    auto file_size = TRY(file->size());

    size_t read_limit = SIZE_MAX;
    auto display_style = DisplayStyle::Hex;
    size_t line_length = BYTES_PER_LINE_HEX;
    size_t group_size = GROUP_SIZE_HEX;
    size_t added_file_offset = 0;
    Optional<ssize_t> seek_offset;

    // todo: put states that can appear together in an if, else if block
    // todo: how to handle invalid combinations ?
    // lazy approach ?

    if (c_include_file_style) {
        line_length = BYTES_PER_LINE_C;
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
        line_length = BYTES_PER_LINE_PLAIN_HEX;
        group_size = GROUP_SIZE_PLAIN_HEX;
    }

    if (line_length_setting.has_value()) {
        line_length = line_length_setting.value();
        if (line_length > 256 || line_length == 0) {
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
        line_length = BYTES_PER_LINE_BITS;
    }

    if (group_size_setting.has_value()) {
        group_size = group_size_setting.value();

        if (little_endian_hexdump) {
            if (!is_power_of_two(group_size)) {
                outln("Group size must be a power of 2 with -e");
                return 1;
            }
        }
    }

    if (read_limit_setting.has_value()) {
        read_limit = read_limit_setting.value();
    }

    if (added_file_offset_setting.has_value()) {
        added_file_offset = added_file_offset_setting.value();
    }

    // TODO: seek relative to current stdin file position
    if (!seek_offset_setting.is_null()) {
        seek_offset = AK::StringUtils::convert_to_int<ssize_t>(seek_offset_setting);
    }

    Array<u8, BUFSIZ> contents;
    Bytes bytes;
    size_t total_bytes_read = 0x0;
    const size_t max_read_size = contents.size() - (contents.size() % line_length);
    bool is_input_remaining = true;

    if (seek_offset.has_value()) {
        auto offset = seek_offset.value();
        total_bytes_read = offset < 0 ? file_size - offset : offset;
    }

    while (is_input_remaining) {
        auto bytes_to_read = max_read_size - bytes.size();

        bytes = contents.span().slice(0, bytes_to_read);
        bytes = TRY(file->read_some(bytes));

        if (bytes.size() < bytes_to_read) {
            is_input_remaining = false;
        }

        while (bytes.size() > 0) {
            auto line_len = bytes.size() > line_length ? line_length : bytes.size();

            if (total_bytes_read + line_len > read_limit) {
                line_len = read_limit - total_bytes_read;
            }

            auto current_line = bytes.slice(0, line_len);
            bytes = bytes.slice(line_len);

            if (autoskip && all_of(bytes, [](auto& b) { return b == 0x0; })) {
                outln("*");
                continue;
            }

            if (display_style != DisplayStyle::CStyle && display_style != DisplayStyle::PlainHex) {
                if (offset_in_decimal) {
                    out("{:08}: ", total_bytes_read + added_file_offset);
                } else {
                    out("{:08x}: ", total_bytes_read + added_file_offset);
                }
            }

            switch (display_style) {
            case DisplayStyle::PlainHex:
            case DisplayStyle::Hex:
                print_line_hex(current_line, line_length, group_size, uppercase_hex);
                break;
            case DisplayStyle::HexLittleEndian:
                print_line_little_endian_hex(current_line, line_length, group_size, uppercase_hex);
                break;
            case DisplayStyle::Bits:
                print_line_bits(current_line, line_length, group_size);
                break;
            case DisplayStyle::CStyle:
                print_line_c_style(current_line);
                break;
            }

            total_bytes_read += line_len;

            if (total_bytes_read >= read_limit) {
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
