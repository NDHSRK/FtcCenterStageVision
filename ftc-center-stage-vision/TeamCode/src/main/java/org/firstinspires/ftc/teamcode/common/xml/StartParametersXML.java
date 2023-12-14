package org.firstinspires.ftc.teamcode.common.xml;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import javax.xml.transform.stream.StreamSource;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class StartParametersXML {

    public static final String TAG = StartParametersXML.class.getSimpleName();
    private static final String FILE_NAME = "StartParameters.xml";
    private final String xmlDirectory;
    private final String xmlFilePath;
    private final Document document;

    private final String robotConfigFilename;
    private final String robotActionFilename;

    private final Node delay_node;
    private int autoStartDelay;

    // IntelliJ only
    /*
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */
    // End IntelliJ only

    public StartParametersXML(String pXMLDirectory) throws ParserConfigurationException, SAXException, IOException {

        // IntelliJ only
        /*
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser.
        dbFactory.setIgnoringElementContentWhitespace(true);
         */
        // End IntelliJ only

        // Android only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        // ONLY works with a validating parser (DTD or schema) dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
        // End Android only

        xmlDirectory = pXMLDirectory;
        xmlFilePath = xmlDirectory + FILE_NAME;

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        document = dBuilder.parse(new File(xmlFilePath));
        Element startParametersRoot = document.getDocumentElement();

        Node config_node = startParametersRoot.getFirstChild();
        config_node = XMLUtils.getNextElement(config_node);
        if (config_node == null || !config_node.getNodeName().equals("robot_config") || config_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'robot_config' not found");

        robotConfigFilename = config_node.getTextContent();

        Node action_node = config_node.getNextSibling();
        action_node = XMLUtils.getNextElement(action_node);
        if (action_node == null || !action_node.getNodeName().equals("robot_action") || action_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'robot_action' not found");

        robotActionFilename = action_node.getTextContent();

        Node local_delay_node = action_node.getNextSibling();
        local_delay_node = XMLUtils.getNextElement(local_delay_node);
        if (local_delay_node == null || !local_delay_node.getNodeName().equals("auto_start_delay") || local_delay_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'auto_start_delay' not found");

        String delayString = local_delay_node.getTextContent();
        try {
            autoStartDelay = Integer.parseInt(delayString);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'auto_start_delay'");
        }

        delay_node = local_delay_node; // preserve "final"

        RobotLog.ii(TAG, "In StartParametersXML; opened and parsed the XML file");
    }

    public String getRobotConfigFilename() {
        return robotConfigFilename;
    }

    public String getRobotActionFilename() {
        return robotActionFilename;
    }

    public int getAutoStartDelay() {
        return autoStartDelay;
    }

    // Replaces the text value of the <auto_start_delay> element.
    public void setAutoStartDelay(int pAutoStartDelay) {
        RobotLog.ii(TAG, "Setting the Autonomous start delay in " + FILE_NAME + " to " + pAutoStartDelay);

        delay_node.setTextContent(Integer.toString(pAutoStartDelay));
        rewriteFile();

        autoStartDelay = pAutoStartDelay;
    }

    // Based on a combination of --
    // https://mkyong.com/java/how-to-modify-xml-file-in-java-dom-parser/#download-source-code
    // https://www.baeldung.com/java-pretty-print-xml
    // using the XSLT method to avoid extra blank lines in the output.
    private void rewriteFile() {
        String xsltFilePath = xmlDirectory + "StandardTransform.xslt";
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(xmlFilePath))) {
            TransformerFactory transformerFactory = TransformerFactory.newInstance();
            //**TODO not supported in Android!! transformerFactory.setAttribute("indent-number", 4);
            Transformer transformer = transformerFactory.newTransformer(new StreamSource(new File(xsltFilePath)));
            transformer.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
            transformer.setOutputProperty(OutputKeys.OMIT_XML_DECLARATION, "yes");
            transformer.setOutputProperty(OutputKeys.INDENT, "yes");
            transformer.transform(new DOMSource(document), new StreamResult(writer));
        } catch (IOException | TransformerException e) {
            throw new AutonomousRobotException(TAG, e.getMessage());
        }
    }

}
