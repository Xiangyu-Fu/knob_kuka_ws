Class type # basic information about task entity
string iri # IRI of OWL individual
string name
string description
string imageUrl

Parameter[] parameters
Precondition[] preconditions
Effect[] effects
PartialOrderingConstraint partialOrderingConstraint

int32 status # TaskStatusEnum.msg
string[] subtasksIri # Set of subtasks. In the interaction_manager tasks without subtasks may be skills?

string currentSubtaskIri # For abstract processes: ""
bool isInferred # False for user-made tasks, even if hidden task parameters such as robot and tool were automatically filled in by the inference engine. True for tasks automatically inserted by the inference engine, e.g., centering and move home tasks.
