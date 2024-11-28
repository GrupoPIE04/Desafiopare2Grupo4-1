package AlgoritimodeDijkastra;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import AlgoritimodeDijkastra.GraphProcessor;
import AlgoritimodeDijkastra.Graph;

public class GraphImporter {
    public static Graph importGraph(List<String> graphData) {
        int vertices = Integer.parseInt(graphData.get(0));
        Graph graph = new Graph(vertices);

        for (int i = 1; i < graphData.size(); i++) {
            String[] parts = graphData.get(i).split(" ");
            int source = Integer.parseInt(parts[0]);
            int target = Integer.parseInt(parts[1]);
            int weight = Integer.parseInt(parts[2]);
            graph.addEdge(source, target, weight);
        }

        return graph;
    }
    public static void main(String[] args) {
        try (Scanner scanner = new Scanner(System.in)) {
            File folder = new File("D:\\faculdade scripts\\algoritimos grafos\\AlgoritimodeDijkastra");
            File[] listOfFiles = folder.listFiles((dir, name) -> name.toLowerCase().endsWith(".gr"));

            if (listOfFiles == null || listOfFiles.length == 0) {
                System.out.println("No .gr files found in the directory.");
                return;
            }

            System.out.println("Select a .gr file to import:");
            for (int i = 0; i < listOfFiles.length; i++) {
                System.out.println((i + 1) + ": " + listOfFiles[i].getName());
            }

            int fileIndex = scanner.nextInt() - 1;
            if (fileIndex < 0 || fileIndex >= listOfFiles.length) {
                System.out.println("Invalid selection.");
                return;
            }

            File selectedFile = listOfFiles[fileIndex];
            List<String> graphData = new ArrayList<>();

            try (Scanner fileScanner = new Scanner(selectedFile)) {
                while (fileScanner.hasNextLine()) {
                    graphData.add(fileScanner.nextLine());
                }
            } catch (FileNotFoundException e) {
                System.out.println("File not found: " + selectedFile.getName());
                return;
            }

            // Send the graph data to another Java file (for example, GraphProcessor)
            GraphProcessor.processGraph(graphData);
        }
    }
}

class GraphProcessor {
    public static void processGraph(List<String> graphData) {
        // Process the graph data
        System.out.println("Processing graph data:");
        for (String line : graphData) {
            System.out.println(line);
        }
    }
}