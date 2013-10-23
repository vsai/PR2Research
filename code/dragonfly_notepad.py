from dragonfly.grammar.grammar     import Grammar
from dragonfly.grammar.context     import AppContext
from dragonfly.grammar.mappingrule import MappingRule
from dragonfly.grammar.elements    import Dictation
from dragonfly.actions.actions     import Key, Text


#---------------------------------------------------------------------------
# Create this module's grammar and the context under which it'll be active.

grammar_context = AppContext(executable="notepad")
grammar = Grammar("notepad_example", context=grammar_context)


#---------------------------------------------------------------------------
# Create a mapping rule which maps things you can say to actions.
#
# Note the relationship between the *mapping* and *extras* keyword
#  arguments.  The extras is a list of Dragonfly elements which are
#  available to be used in the specs of the mapping.  In this example
#  the Dictation("text")* extra makes it possible to use "<text>"
#  within a mapping spec and "%(text)s" within the associated action.

example_rule = MappingRule(
    name="example",                    # The name of the rule.
    mapping={                          # The mapping dict: spec -> action.
             "save [file]":            Key("c-s"),
             "save [file] as":         Key("a-f, a"),
             "save [file] as <text>":  Key("a-f, a/20") + Text("%(text)s"),
             "find <text>":            Key("c-f/20") + Text("%(text)s\n"),
            },
    extras=[                           # Special elements in the specs of the mapping.
            Dictation("text"),
           ],
    )

# Add the action rule to the grammar instance.
grammar.add_rule(example_rule)


#---------------------------------------------------------------------------
# Load the grammar instance and define how to unload it.

grammar.load()

# Unload function which will be called by natlink at unload time.
def unload():
    global grammar
    if grammar: grammar.unload()
    grammar = None
