string iri # The IRI of the object or data property. For example: <http://kb.local/rest/kb/smerobotics.owl#endPose>.
string name # human-readable simple name
string description # human-readable description

string[] range # (Union of ranges.) Empty means no restrictions. XSD datatypes and OWL classes. Examples: <http://www.w3.org/2001/XMLSchema#double>, <http://kb.local/rest/kb/smerobotics.owl#PhysicalObject>, <http://kb.local/rest/kb/smerobotics.owl#WoodenPanelType>.

Value value

bool isOptional # Indicates whether or not this parameter is optional during teaching.
int32 hiddenLevel # Indicates whether or not this parameter is hidden from the user in the GUI when teaching or showing the execution status, e.g. workcellIri, robotIri, gripperIri. HiddenLevel is a range: 0 (always show), 1 (hide during teaching), 2 (normally hide during execution), 3 (advanced view during execution). Hidden parameters may be changed or added during the execution, e.g., depending on a laser distance sensor?

string quantityName # Often: "". Examples: "length", "speed", "force".
string quantitySymbol # Often: "". Examples: "L", "v", "F".
string unitName # Often: "". Examples: "metre", "metre per second", "newton".
string unitSymbol # Often: "". Examples: "m", "m/s", "N".
