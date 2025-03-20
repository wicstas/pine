#include <pine/core/earley.h>
#include <pine/core/log.h>

#include <psl/type_traits.h>
#include <psl/optional.h>
#include <psl/map.h>

namespace pine {

struct Parser {
  struct State {
    State() = default;
    State(uint16_t origin, uint8_t rule_index, uint8_t position)
        : origin{origin}, rule_index{rule_index}, position{position} {
    }

    bool no_prev() const {
      return prev_loc == invalid_pre_loc;
    }

    uint16_t origin = 0;
    uint8_t rule_index = 0;
    uint8_t position = 0;
    static constexpr uint16_t invalid_pre_loc = uint16_t(-1);
    uint16_t prev_loc = invalid_pre_loc;
    uint16_t prev_index = 0;
    enum class CompleterType : uint16_t { Null, State, Token } completer_type = CompleterType::Null;
    uint16_t completer_index = 0;
  };
  using States = psl::vector<State>;

  Parser(const Grammar& grammar, const psl::vector<EarleyNode>& tokens)
      : grammar{grammar}, tokens{tokens} {
  }

  psl::optional<EarleyNode> parse(const psl::string& root_sym) {
    states_list = psl::vector<States>(size(tokens) + 1);

    set_initial_states(root_sym);

    for (size_t position = 0; position <= size(tokens); position++) {
      auto& states = states_list[position];

      for (size_t i = 0; i < size(states); i++) {
        const auto state = states[i];

        if (state.position != 0 && (prev_item(state).qualifier == Qualifier::Many ||
                                    prev_item(state).qualifier == Qualifier::OneOrMore)) {
          auto new_state = state;
          new_state.position--;
          add_state(states, new_state);
        }

        if (!completed(state) && (next_item(state).qualifier == Qualifier::Maybe ||
                                  next_item(state).qualifier == Qualifier::Many)) {
          auto new_state = state;
          new_state.position++;
          add_state(states, new_state);
        }

        if (completed(state)) {
          for (size_t j = 0; j < states_list[state.origin].size(); j++) {
            auto prev_state = states_list[state.origin][j];
            if (!completed(prev_state) && sym(state) == next_item(prev_state).sym) {
              prev_state.position++;
              prev_state.prev_loc = state.origin;
              prev_state.prev_index = j;
              prev_state.completer_type = State::CompleterType::State;
              prev_state.completer_index = i;
              add_state(states, prev_state);
            }
          }
        } else {
          const auto range = grammar.sym2index.equal_range(next_item(state).sym);
          for (auto it = range.first; it != range.second; it++)
            add_state(states, State(position, it->second, 0));

          for (size_t j = 0; j < size(states); j++) {
            if (completed(states[j]) && sym(states[j]) == next_item(state).sym) {
              auto new_state = state;
              new_state.position++;
              new_state.prev_loc = position;
              new_state.prev_index = i;
              new_state.completer_type = State::CompleterType::State;
              new_state.completer_index = j;
              add_state(states, new_state);
            }
          }
        }
      }

      if (position != size(tokens))
        for (size_t i = 0; i < size(states); i++) {
          auto state = states[i];
          if (!completed(state) && next_item(state).sym == tokens[position].sym) {
            state.position++;
            state.prev_loc = position;
            state.prev_index = i;
            state.completer_type = State::CompleterType::Token;
            state.completer_index = position;
            states_list[position + 1].push_back(state);
          }
        }
    }

    const auto final_loc = states_list.size() - 1;
    for (size_t i = 0; i < states_list[final_loc].size(); i++) {
      const auto& state = states_list[final_loc][i];
      if (completed(state) && state.origin == 0 && sym(state) == grammar.rules[0].sym) {
        return backtrace(final_loc, i);
      }
    }

    return psl::nullopt;
  }

  EarleyNode backtrace(size_t state_loc, size_t state_index) {
    auto node = EarleyNode{sym(get_state(state_loc, state_index)), {}};
    while (!get_state(state_loc, state_index).no_prev()) {
      const auto& state = get_state(state_loc, state_index);
      const auto& prev_state = get_state(state.prev_loc, state.prev_index);
      auto superficial = next_item(prev_state).superficial;

      if (!superficial) {
        if (state.completer_type == State::CompleterType::Token)
          node.children.push_back(tokens[state.completer_index]);
        else if (state.completer_type == State::CompleterType::State)
          node.children.push_back(backtrace(state_loc, state.completer_index));
        else
          SEVERE("Should never reach here");
      }
      state_loc = state.prev_loc;
      state_index = state.prev_index;
    }
    psl::reverse(node.children);

    return node;
  }

  State& get_state(size_t state_loc, size_t state_index) {
    CHECK_LT(state_loc, states_list.size());
    CHECK_LT(state_index, states_list[state_loc].size());
    return states_list[state_loc][state_index];
  }
  bool completed(const State& state) {
    return size(grammar.rules[state.rule_index].expr) == state.position;
  };
  psl::string sym(const State& state) {
    return grammar.rules[state.rule_index].sym;
  };
  GrammarItem next_item(const State& state) {
    CHECK(!completed(state));
    return grammar.rules[state.rule_index].expr[state.position];
  };
  GrammarItem prev_item(const State& state) {
    CHECK(state.position != 0);
    return grammar.rules[state.rule_index].expr[state.position - 1];
  };
  void set_initial_states(const psl::string& root_sym) {
    const auto range = grammar.sym2index.equal_range(root_sym);
    for (auto it = range.first; it != range.second; it++)
      states_list[0].push_back(State(0, it->second, 0));
  };

  bool add_state(States& states, const State& state) {
    if (psl::find_if(states, [&](const State& rhs) {
          return rhs.origin == state.origin && rhs.rule_index == state.rule_index &&
                 rhs.position == state.position;
        }) == end(states)) {
      states.push_back(state);
      return true;
    } else {
      return false;
    }
  };

private:
  const Grammar& grammar;
  const psl::vector<EarleyNode>& tokens;
  psl::vector<States> states_list;
};

psl::optional<EarleyNode> parse(const Grammar& grammar, const psl::vector<EarleyNode>& tokens,
                                const psl::string& root_sym) {
  auto parser = Parser(grammar, tokens);
  return parser.parse(root_sym);
}

Object generate_impl(const Generator& generator, const EarleyNode& node) {
  auto build = [&](auto& build, const EarleyNode& node) -> Object {
    if (node.sym.subview(0, 6) == "@maybe") {
      if (node.children.size() == 0)
        return generator.find_model(node.sym.substr(7))->make_null_optional();
      else
        return build(build, node.children[0]).make_optional();
    } else if (node.sym.subview(0, 5) == "@many") {
      if (node.children.size() == 0) {
        return generator.find_model(node.sym.substr(6))->make_empty_vector();
      } else {
        auto elements = psl::vector<Object>{};
        for (const auto& child : node.children)
          elements.push_back(build(build, child));
        auto vector = elements[0].make_empty_vector();
        for (auto& elem : elements)
          elem.move_append(vector);
        return vector;
      }
    } else if (node.sym.subview(0, 10) == "@oneOrMore") {
      CHECK_GE(node.children.size(), 1);
      auto elements = psl::vector<Object>{};
      for (const auto& child : node.children)
        elements.push_back(build(build, child));
      auto vector = elements[0].make_empty_vector();
      for (auto& elem : elements)
        elem.move_append(vector);
      return vector;
    }
    auto model = generator.find_model(node.sym);

    psl::vector<Object> args;

    if (!model->use_string)
      for (const auto& child : node.children)
        args.push_back(build(build, child));

    if (model->use_string && size(node.children) != 1)
      SEVERE(
          "Model binded with `UseString` tag requires exactly one child in the"
          "corresponding node");

    auto ptr = model->use_string ? model->construct(node.children[0].sym) : model->construct(args);
    if (!ptr) {
      auto argsTypesName = psl::string();
      for (const auto& arg : args)
        argsTypesName += arg.type_name() + psl::string(" ");
      if (model->use_string)
        SEVERE("Cannot construct type ", model->type_name(), " from argument of type psl::string");
      else
        SEVERE("Cannot construct type ", model->type_name(), " from arguments of types ",
              argsTypesName);
    }

    return *ptr;
  };

  return build(build, node);
}

}  // namespace pine
