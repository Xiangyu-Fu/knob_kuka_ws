Class type # basic information about process entity
string iri # IRI of OWL individual
string name
string description
string imageUrl

int32 status # TaskStatusEnum.msg
string[] subtasksIri # (Sorted) list of top-level tasks in the process.

string currentSubtaskIri # For abstract processes: ""
string workcellIri # For abstract processes: ""
